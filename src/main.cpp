#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#ifndef NODE_ID_VALUE
#define NODE_ID_VALUE "esp32_bedroom"
#endif

constexpr int STATUS_LED_PIN = 2;
constexpr int RADAR_RX_PIN = 16;
constexpr int RADAR_TX_PIN = 17;
constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t RADAR_BAUD = 256000;
constexpr size_t FRAME_HEADER_LENGTH = 4;
constexpr size_t FRAME_DATA_LENGTH = 24;
constexpr size_t FRAME_FOOTER_LENGTH = 2;
constexpr size_t FRAME_LENGTH =
    FRAME_HEADER_LENGTH + FRAME_DATA_LENGTH + FRAME_FOOTER_LENGTH;
constexpr size_t TARGET_COUNT = 3;
constexpr size_t TARGET_GROUP_LENGTH = 8;
constexpr uint32_t DASHBOARD_INTERVAL_MS = 1000;
constexpr uint32_t STATUS_INTERVAL_MS = 2000;
constexpr uint32_t ACTIVE_DATA_TIMEOUT_MS = 1000;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 5000;
constexpr uint32_t NETWORK_FAILURE_BACKOFF_MS = 80;
constexpr int RADAR_RANGE_X_CM = 300;
constexpr int RADAR_RANGE_Y_CM = 600;
constexpr int RADAR_GRID_WIDTH = 41;
constexpr int RADAR_GRID_HEIGHT = 16;
constexpr char WIFI_SSID[] = "MyOptimum 0a6449";
constexpr char WIFI_PASSWORD[] = "2982-pewter-40";
constexpr char NODE_ID[] = NODE_ID_VALUE;
constexpr char SERVER_BASE_URL[] = "http://192.168.1.128:8000";

constexpr uint8_t FRAME_HEADER[FRAME_HEADER_LENGTH] = {0xAA, 0xFF, 0x03, 0x00};
constexpr uint8_t FRAME_FOOTER[FRAME_FOOTER_LENGTH] = {0x55, 0xCC};

enum class FrameState : uint8_t {
  SeekingHeader,
  ReadingPayload,
  ReadingFooter
};

struct TargetInfo {
  bool valid = false;
  float x_cm = 0.0f;
  float y_cm = 0.0f;
  float speed_cm_s = 0.0f;
  uint16_t gate_mm = 0;
};

struct RadarSnapshot {
  uint32_t ts_ms = 0;
  TargetInfo targets[TARGET_COUNT];
};

FrameState frameState = FrameState::SeekingHeader;
uint8_t headerIndex = 0;
uint8_t payload[FRAME_DATA_LENGTH] = {};
size_t payloadIndex = 0;
uint8_t footerIndex = 0;

TargetInfo latestTargets[TARGET_COUNT];
uint32_t totalBytesReceived = 0;
uint32_t totalFramesReceived = 0;
uint32_t totalFramesDropped = 0;
uint32_t lastByteMillis = 0;
uint32_t lastFrameMillis = 0;
uint32_t lastDashboardMillis = 0;
uint32_t lastStatusMillis = 0;
uint32_t lastBlinkMillis = 0;
bool rawDumpEnabled = false;
bool plotStreamEnabled = false;
bool dashboardEnabled = false;
uint32_t lastWifiAttemptMillis = 0;
uint32_t lastPostMillis = 0;
int lastPostStatusCode = 0;
uint32_t postSuccessCount = 0;
uint32_t postFailureCount = 0;
QueueHandle_t radarSnapshotQueue = nullptr;
String serverUrl;

void ensureWiFiConnected();
void queueSnapshotForUpload(uint32_t now);
void radarUploadTask(void* parameter);

void printHexByte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printFrameHex(const uint8_t* data, size_t length) {
  Serial.print("frame ");
  for (size_t i = 0; i < length; ++i) {
    printHexByte(data[i]);
    Serial.print(i + 1 == length ? '\n' : ' ');
  }
}

void updateStatusLed(uint32_t now) {
  const bool radarActive =
      totalBytesReceived > 0 && (now - lastByteMillis) < ACTIVE_DATA_TIMEOUT_MS;

  if (radarActive) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }

  if (now - lastBlinkMillis >= 250) {
    lastBlinkMillis = now;
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
  }
}

void resetFrameReader() {
  frameState = FrameState::SeekingHeader;
  headerIndex = 0;
  payloadIndex = 0;
  footerIndex = 0;
}

float decodeSignedMagnitudeCm(uint8_t lowByte, uint8_t highByte) {
  const uint16_t magnitude =
      static_cast<uint16_t>((highByte & 0x7F) << 8) | lowByte;
  float value = magnitude / 10.0f;
  if ((highByte & 0x80) == 0) {
    value = -value;
  }
  return value;
}

float decodeYOffsetCm(uint8_t lowByte, uint8_t highByte) {
  const int32_t rawValue =
      static_cast<int32_t>((static_cast<uint16_t>(highByte) << 8) | lowByte);
  return (rawValue - 0x8000) / 10.0f;
}

TargetInfo decodeTarget(const uint8_t* group) {
  TargetInfo target;

  bool anyNonZero = false;
  for (size_t i = 0; i < TARGET_GROUP_LENGTH; ++i) {
    if (group[i] != 0x00) {
      anyNonZero = true;
      break;
    }
  }

  if (!anyNonZero) {
    return target;
  }

  target.valid = true;
  target.x_cm = decodeSignedMagnitudeCm(group[0], group[1]);
  target.y_cm = decodeYOffsetCm(group[2], group[3]);
  target.speed_cm_s = decodeSignedMagnitudeCm(group[4], group[5]);
  target.gate_mm = static_cast<uint16_t>(group[7] << 8) | group[6];
  return target;
}

void decodeFrame(const uint8_t* data) {
  for (size_t i = 0; i < TARGET_COUNT; ++i) {
    latestTargets[i] = decodeTarget(&data[i * TARGET_GROUP_LENGTH]);
  }
}

void printTargetLine(size_t index, const TargetInfo& target) {
  if (!target.valid) {
    Serial.printf("T%u: no target\n", static_cast<unsigned>(index + 1));
    return;
  }

  Serial.printf("T%u: X=%7.1f cm  Y=%7.1f cm  Spd=%7.1f cm/s  Gate=%4u mm\n",
                static_cast<unsigned>(index + 1), target.x_cm, target.y_cm,
                target.speed_cm_s, target.gate_mm);
}

void emitPlotLine() {
  Serial.print("PLOT");
  for (size_t i = 0; i < TARGET_COUNT; ++i) {
    const TargetInfo& target = latestTargets[i];
    Serial.printf(",%u,%.1f,%.1f,%.1f,%u", target.valid ? 1U : 0U, target.x_cm,
                  target.y_cm, target.speed_cm_s, target.gate_mm);
  }
  Serial.println();
}

String buildRadarPayloadJson(const RadarSnapshot& snapshot) {
  String body;
  body.reserve(256);
  body += "{\"node_id\":\"";
  body += NODE_ID;
  body += "\",\"ts_ms\":";
  body += String(snapshot.ts_ms);
  body += ",\"targets\":[";

  bool wroteTarget = false;
  for (size_t i = 0; i < TARGET_COUNT; ++i) {
    const TargetInfo& target = snapshot.targets[i];
    if (!target.valid) {
      continue;
    }

    if (wroteTarget) {
      body += ',';
    }

    body += "{\"target_id\":";
    body += String(i + 1);
    body += ",\"x_m\":";
    body += String(target.x_cm / 100.0f, 3);
    body += ",\"y_m\":";
    body += String(target.y_cm / 100.0f, 3);
    body += ",\"speed_m_s\":";
    body += String(target.speed_cm_s / 100.0f, 3);
    body += ",\"gate_m\":";
    body += String(target.gate_mm / 1000.0f, 3);
    body += '}';
    wroteTarget = true;
  }

  body += "]}";
  return body;
}

void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const uint32_t now = millis();
  if (lastWifiAttemptMillis != 0 &&
      (now - lastWifiAttemptMillis) < WIFI_RETRY_INTERVAL_MS) {
    return;
  }

  lastWifiAttemptMillis = now;
  Serial.printf("[wifi] connecting to \"%s\"\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void queueSnapshotForUpload(uint32_t now) {
  if (radarSnapshotQueue == nullptr) {
    return;
  }

  RadarSnapshot snapshot;
  snapshot.ts_ms = now;
  for (size_t i = 0; i < TARGET_COUNT; ++i) {
    snapshot.targets[i] = latestTargets[i];
  }

  xQueueOverwrite(radarSnapshotQueue, &snapshot);
}

bool postSnapshotToServer(const RadarSnapshot& snapshot) {
  if (WiFi.status() != WL_CONNECTED) {
    lastPostStatusCode = -1;
    return false;
  }

  HTTPClient http;
  http.setConnectTimeout(1200);
  http.setTimeout(1200);

  if (!http.begin(serverUrl)) {
    lastPostStatusCode = -2;
    postFailureCount++;
    Serial.println("[http] failed to begin request");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  const String body = buildRadarPayloadJson(snapshot);
  const int statusCode = http.POST(body);
  lastPostStatusCode = statusCode;
  lastPostMillis = millis();

  if (statusCode > 0 && statusCode < 400) {
    postSuccessCount++;
  } else {
    postFailureCount++;
    Serial.printf("[http] POST failed: %d\n", statusCode);
  }

  http.end();
  return statusCode > 0 && statusCode < 400;
}

void radarUploadTask(void* parameter) {
  (void)parameter;
  RadarSnapshot snapshot;

  for (;;) {
    if (radarSnapshotQueue == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    const BaseType_t received =
        xQueueReceive(radarSnapshotQueue, &snapshot, pdMS_TO_TICKS(250));
    ensureWiFiConnected();

    if (received != pdPASS) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    const bool ok = postSnapshotToServer(snapshot);
    if (!ok) {
      vTaskDelay(pdMS_TO_TICKS(NETWORK_FAILURE_BACKOFF_MS));
    }
  }
}

void renderRadarGrid() {
  char grid[RADAR_GRID_HEIGHT][RADAR_GRID_WIDTH + 1];
  const int centerX = RADAR_GRID_WIDTH / 2;
  const int bottomY = RADAR_GRID_HEIGHT - 1;

  for (int row = 0; row < RADAR_GRID_HEIGHT; ++row) {
    for (int col = 0; col < RADAR_GRID_WIDTH; ++col) {
      grid[row][col] = ' ';
    }
    grid[row][RADAR_GRID_WIDTH] = '\0';
  }

  for (int row = 0; row < RADAR_GRID_HEIGHT; ++row) {
    grid[row][centerX] = '|';
  }

  for (int row = 0; row < RADAR_GRID_HEIGHT - 1; row += 3) {
    for (int col = 0; col < RADAR_GRID_WIDTH; ++col) {
      if (col == centerX) {
        continue;
      }
      grid[row][col] = '.';
    }
  }

  for (int col = 0; col < RADAR_GRID_WIDTH; ++col) {
    grid[bottomY][col] = '-';
  }

  grid[bottomY][centerX] = '^';

  for (size_t i = 0; i < TARGET_COUNT; ++i) {
    const TargetInfo& target = latestTargets[i];
    if (!target.valid) {
      continue;
    }

    const float normalizedX =
        constrain(target.x_cm, -RADAR_RANGE_X_CM, RADAR_RANGE_X_CM) /
        static_cast<float>(RADAR_RANGE_X_CM);
    const float normalizedY =
        constrain(target.y_cm, 0.0f, static_cast<float>(RADAR_RANGE_Y_CM)) /
        static_cast<float>(RADAR_RANGE_Y_CM);

    int col = centerX + static_cast<int>(roundf(normalizedX * centerX));
    int row =
        bottomY - 1 - static_cast<int>(roundf(normalizedY * (bottomY - 1)));

    col = constrain(col, 0, RADAR_GRID_WIDTH - 1);
    row = constrain(row, 0, RADAR_GRID_HEIGHT - 2);
    grid[row][col] = static_cast<char>('1' + i);
  }

  Serial.printf("X range: -%dcm .. +%dcm, Y range: 0..%dcm\n", RADAR_RANGE_X_CM,
                RADAR_RANGE_X_CM, RADAR_RANGE_Y_CM);
  for (int row = 0; row < RADAR_GRID_HEIGHT; ++row) {
    Serial.println(grid[row]);
  }
}

void renderDashboard(uint32_t now) {
  Serial.write(27);
  Serial.print("[2J");
  Serial.write(27);
  Serial.print("[H");

  Serial.println("LD2450 Radar View");
  Serial.println("=================");
  Serial.printf("Frames: %lu  Bytes: %lu  Dropped: %lu  Last frame: %lums ago\n",
                static_cast<unsigned long>(totalFramesReceived),
                static_cast<unsigned long>(totalBytesReceived),
                static_cast<unsigned long>(totalFramesDropped),
                totalFramesReceived == 0
                    ? 0UL
                    : static_cast<unsigned long>(now - lastFrameMillis));
  Serial.printf(
      "WiFi: %s  IP: %s  HTTP: %d  POST ok/fail: %lu/%lu  Last POST: %lums ago\n",
      WiFi.status() == WL_CONNECTED ? "connected" : "offline",
      WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-",
      lastPostStatusCode, static_cast<unsigned long>(postSuccessCount),
      static_cast<unsigned long>(postFailureCount),
      lastPostMillis == 0 ? 0UL
                          : static_cast<unsigned long>(now - lastPostMillis));
  Serial.println("Commands: h=help  r=toggle raw frame dump");
  Serial.println();

  renderRadarGrid();

  Serial.println();
  for (size_t i = 0; i < TARGET_COUNT; ++i) {
    printTargetLine(i, latestTargets[i]);
  }
}

void printHelp() {
  Serial.println("LD2450 commands:");
  Serial.println("  h : print help");
  Serial.println("  r : toggle raw frame dump");
  Serial.println("  p : toggle PLOT serial stream");
  Serial.println("  v : toggle dashboard view");
}

void handleConsoleInput() {
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      return;
    }

    const char command = static_cast<char>(tolower(value));
    if (command == 'h') {
      printHelp();
    } else if (command == 'r') {
      rawDumpEnabled = !rawDumpEnabled;
      Serial.printf("Raw frame dump: %s\n", rawDumpEnabled ? "ON" : "OFF");
    } else if (command == 'p') {
      plotStreamEnabled = !plotStreamEnabled;
      Serial.printf("PLOT stream: %s\n", plotStreamEnabled ? "ON" : "OFF");
    } else if (command == 'v') {
      dashboardEnabled = !dashboardEnabled;
      Serial.printf("Dashboard: %s\n", dashboardEnabled ? "ON" : "OFF");
    }
  }
}

void handleFrameByte(uint8_t value) {
  switch (frameState) {
    case FrameState::SeekingHeader:
      if (value == FRAME_HEADER[headerIndex]) {
        headerIndex++;
        if (headerIndex == FRAME_HEADER_LENGTH) {
          frameState = FrameState::ReadingPayload;
          payloadIndex = 0;
        }
      } else {
        headerIndex = value == FRAME_HEADER[0] ? 1 : 0;
      }
      break;

    case FrameState::ReadingPayload:
      payload[payloadIndex++] = value;
      if (payloadIndex == FRAME_DATA_LENGTH) {
        frameState = FrameState::ReadingFooter;
        footerIndex = 0;
      }
      break;

    case FrameState::ReadingFooter:
      if (value == FRAME_FOOTER[footerIndex]) {
        footerIndex++;
        if (footerIndex == FRAME_FOOTER_LENGTH) {
          totalFramesReceived++;
          lastFrameMillis = millis();
          decodeFrame(payload);
          if (plotStreamEnabled) {
            emitPlotLine();
          }
          queueSnapshotForUpload(lastFrameMillis);

          if (rawDumpEnabled) {
            uint8_t frame[FRAME_LENGTH];
            memcpy(frame, FRAME_HEADER, FRAME_HEADER_LENGTH);
            memcpy(frame + FRAME_HEADER_LENGTH, payload, FRAME_DATA_LENGTH);
            memcpy(frame + FRAME_HEADER_LENGTH + FRAME_DATA_LENGTH, FRAME_FOOTER,
                   FRAME_FOOTER_LENGTH);
            printFrameHex(frame, FRAME_LENGTH);
          }

          resetFrameReader();
        }
      } else {
        totalFramesDropped++;
        resetFrameReader();
        handleFrameByte(value);
      }
      break;
  }
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(DEBUG_BAUD);
  delay(500);
  Serial.println();
  Serial.println("LD2450 UART2 parser + radar view");
  Serial.println("ESP32 RX2(GPIO16) <- LD2450 TX");
  Serial.println("ESP32 TX2(GPIO17) -> LD2450 RX");
  Serial.println("Radar UART: 256000 8N1");
  serverUrl = String(SERVER_BASE_URL) + "/api/radar/" + NODE_ID;
  Serial.printf("WiFi SSID: %s\n", WIFI_SSID);
  Serial.printf("Node ID: %s\n", NODE_ID);
  Serial.printf("Server URL: %s\n", serverUrl.c_str());
  printHelp();

  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  radarSnapshotQueue = xQueueCreate(1, sizeof(RadarSnapshot));
  xTaskCreatePinnedToCore(radarUploadTask, "radarUpload", 6144, nullptr, 1,
                          nullptr, 0);
  ensureWiFiConnected();

  const uint32_t now = millis();
  lastStatusMillis = now;
  lastBlinkMillis = now;
  lastDashboardMillis = now;
}

void loop() {
  const uint32_t now = millis();
  ensureWiFiConnected();
  handleConsoleInput();

  while (Serial2.available() > 0) {
    const int value = Serial2.read();
    if (value < 0) {
      break;
    }

    if (totalBytesReceived == 0) {
      Serial.println("First radar byte received.");
    }

    totalBytesReceived++;
    lastByteMillis = now;
    handleFrameByte(static_cast<uint8_t>(value));
  }

  if (dashboardEnabled && (now - lastDashboardMillis >= DASHBOARD_INTERVAL_MS)) {
    lastDashboardMillis = now;
    renderDashboard(now);
  }

  if (now - lastStatusMillis >= STATUS_INTERVAL_MS) {
    lastStatusMillis = now;
    Serial.printf("[status] bytes=%lu frames=%lu dropped=%lu wifi=%s ip=%s http=%d last_byte_ms_ago=%lu\n",
                  static_cast<unsigned long>(totalBytesReceived),
                  static_cast<unsigned long>(totalFramesReceived),
                  static_cast<unsigned long>(totalFramesDropped),
                  WiFi.status() == WL_CONNECTED ? "up" : "down",
                  WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str()
                                                : "-",
                  lastPostStatusCode,
                  totalBytesReceived == 0
                      ? 0UL
                      : static_cast<unsigned long>(now - lastByteMillis));
  }

  updateStatusLed(now);
}
