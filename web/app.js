const floorplanGrid = document.getElementById("floorplan-grid");
const exportLayoutButton = document.getElementById("export-layout");
const clearSelectionButton = document.getElementById("clear-selection");

const serverStatusEl = document.getElementById("server-status");
const lastUpdateEl = document.getElementById("last-update");
const nodeCountEl = document.getElementById("node-count");
const onlineCountEl = document.getElementById("online-count");
const targetCountEl = document.getElementById("target-count");
const freshestAgeEl = document.getElementById("freshest-age");
const scaleLabelEl = document.getElementById("scale-label");
const nodeHealthList = document.getElementById("node-health-list");
const lidarViewerList = document.getElementById("lidar-viewer-list");

const inspectorTitle = document.getElementById("inspector-title");
const inspectorSubtitle = document.getElementById("inspector-subtitle");
const inspectorLabel = document.getElementById("inspector-label");
const inspectorNode = document.getElementById("inspector-node");
const inspectorX = document.getElementById("inspector-x");
const inspectorY = document.getElementById("inspector-y");
const inspectorHeadingRange = document.getElementById("inspector-heading-range");
const inspectorHeading = document.getElementById("inspector-heading");
const inspectorRange = document.getElementById("inspector-range");
const inspectorFov = document.getElementById("inspector-fov");
const inspectorRangeValue = document.getElementById("inspector-range-value");
const inspectorFovValue = document.getElementById("inspector-fov-value");
const inspectorRangeField = document.getElementById("inspector-range-field");
const inspectorFovField = document.getElementById("inspector-fov-field");
const inspectorMirror = document.getElementById("inspector-mirror");
const inspectorPriority = document.getElementById("inspector-priority");
const bringToFrontButton = document.getElementById("bring-to-front");
const sendToBackButton = document.getElementById("send-to-back");
const duplicateRadarButton = document.getElementById("duplicate-radar");
const removeRadarButton = document.getElementById("remove-radar");
const inspectorPreview = document.getElementById("inspector-preview");
const inspectorNodeHealth = document.getElementById("inspector-node-health");

const CLIENT_ID = `client_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`;
const STATIC_VERSION = "20260418a";
const LAYOUT_SAVE_DELAY_MS = 180;
const NODE_ONLINE_MAX_AGE_MS = 1500;
const LIDAR_ONLINE_MAX_AGE_MS = 1800;
const HEALTH_TICK_MS = 500;
const DEMO_TICK_MS = 80;
const LIDAR_VIEW_RANGE_M = 12;

const palette = {
  imageOverlay: "rgba(255,255,255,0.18)",
  nodeBody: "#1568a8",
  nodeSelected: "#0e8f73",
  nodeCone: "rgba(21, 104, 168, 0.18)",
  target: "#eb4f37",
  targetHalo: "rgba(235, 79, 55, 0.22)",
  targetLine: "rgba(235, 79, 55, 0.42)",
  handle: "#ffd166",
  text: "#14231d",
};

const state = { meta: {}, nodes: {}, last_update_ms: 0 };
const lidarState = {};

let appConfig = { global_scale: { pixels_per_meter: 44 }, floors: [] };
let activeFloorId = "";
let selectedRadar = { floorId: "", radarId: "" };
const floorViews = new Map();
const lidarViews = new Map();
let layoutSaveTimer = null;
let lastNodeSignature = "";
let lastLidarSignature = "";

function isHiddenNodeId(nodeId) {
  return typeof nodeId === "string" && nodeId.endsWith("__baseline");
}

function visibleNodeEntries() {
  return Object.entries(state.nodes).filter(([nodeId]) => !isHiddenNodeId(nodeId));
}

function visibleNodes() {
  return visibleNodeEntries().map(([, node]) => node);
}

function visibleLidarNodeIds() {
  return Object.keys(lidarState).sort();
}

function renderFloorViewsOnly() {
  floorViews.forEach((view) => view.render());
}

function deepClone(value) {
  return JSON.parse(JSON.stringify(value));
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function floorPixelsPerMeter(floor) {
  const floorScale = Number(floor?.pixels_per_meter);
  return Number.isFinite(floorScale) && floorScale > 0
    ? floorScale
    : appConfig.global_scale.pixels_per_meter;
}

function headingVectors(headingDeg) {
  const headingRad = (headingDeg * Math.PI) / 180;
  return {
    forward: { x: Math.sin(headingRad), y: Math.cos(headingRad) },
    right: { x: Math.cos(headingRad), y: -Math.sin(headingRad) },
  };
}

function rotateVector(vector, angleRad) {
  return {
    x: vector.x * Math.cos(angleRad) - vector.y * Math.sin(angleRad),
    y: vector.x * Math.sin(angleRad) + vector.y * Math.cos(angleRad),
  };
}

function planWidthMeters(floor) {
  return floor.image_width_px / floorPixelsPerMeter(floor);
}

function planHeightMeters(floor) {
  return floor.image_height_px / floorPixelsPerMeter(floor);
}

function normalizeFloorPriorities(floor) {
  const sorted = [...floor.radars].sort((a, b) => {
    const priorityA = Number.isFinite(Number(a.priority)) ? Number(a.priority) : Number.MIN_SAFE_INTEGER;
    const priorityB = Number.isFinite(Number(b.priority)) ? Number(b.priority) : Number.MIN_SAFE_INTEGER;
    if (priorityA !== priorityB) return priorityA - priorityB;
    return String(a.id).localeCompare(String(b.id));
  });

  sorted.forEach((radar, index) => {
    radar.priority = index;
  });
  floor.radars = sorted;
}

function normalizeLayoutPriorities(layout) {
  layout.floors.forEach((floor) => normalizeFloorPriorities(floor));
}

function orderedRadars(floor) {
  return [...floor.radars].sort((a, b) => Number(a.priority || 0) - Number(b.priority || 0));
}

function maxPriority(floor) {
  if (floor.radars.length === 0) return 0;
  return Math.max(...floor.radars.map((radar) => Number(radar.priority || 0)));
}

function isOmniSensor(radar) {
  return radar.coverage_mode === "omni" || Number(radar.fov_deg) >= 360;
}

function createRadarTemplate(floor) {
  return {
    id: `radar_${floor.id}_${Date.now()}`,
    label: `Radar ${floor.radars.length + 1}`,
    node_id: "",
    x_m: planWidthMeters(floor) / 2,
    y_m: planHeightMeters(floor) / 2,
    heading_deg: 180,
    mirror_x: false,
    range_m: 6,
    fov_deg: 120,
    color: "#1568a8",
    priority: maxPriority(floor) + 1,
  };
}

function createLidarTemplate(floor) {
  return {
    id: `lidar_${floor.id}_${Date.now()}`,
    label: `Lidar ${floor.radars.length + 1}`,
    node_id: "",
    x_m: planWidthMeters(floor) / 2,
    y_m: planHeightMeters(floor) / 2,
    heading_deg: 0,
    mirror_x: false,
    range_m: 15,
    fov_deg: 360,
    color: "#0e8f73",
    coverage_mode: "omni",
    priority: maxPriority(floor) + 1,
  };
}

function generateSimulatedTargets(radar) {
  const t = Date.now() / 1000;
  const primaryAngle = t * 0.55;
  const primaryRadius = 2.8 + Math.sin(t * 0.27) * 0.85;
  const secondaryAngle = primaryAngle + Math.PI * 0.92;
  const secondaryRadius = 1.4 + Math.sin(t * 0.43) * 0.45;

  return [
    {
      target_id: 1,
      x_m: Math.cos(primaryAngle) * primaryRadius,
      y_m: Math.sin(primaryAngle) * primaryRadius,
      speed_m_s: 0.9,
      gate_m: radar.range_m,
    },
    {
      target_id: 2,
      x_m: Math.cos(secondaryAngle) * secondaryRadius,
      y_m: Math.sin(secondaryAngle) * secondaryRadius,
      speed_m_s: 0.4,
      gate_m: radar.range_m,
    },
  ];
}

function isTargetVisibleForSensor(radar, target) {
  const localX = Number(target.x_m || 0);
  const localY = Number(target.y_m || 0);
  const distance = Math.hypot(localX, localY);
  if (distance > Number(radar.range_m || 0)) {
    return false;
  }

  if (isOmniSensor(radar)) {
    return true;
  }

  const halfFov = Number(radar.fov_deg || 0) / 2;
  const angleDeg = (Math.atan2(localX, localY) * 180) / Math.PI;
  return Math.abs(angleDeg) <= halfFov;
}

function getLidarScan(nodeId) {
  return nodeId ? lidarState[nodeId] || null : null;
}

function globalToSensorLocal(radar, point) {
  const { forward, right } = headingVectors(radar.heading_deg);
  const dx = Number(point.x) - Number(radar.x_m);
  const dy = Number(point.y) - Number(radar.y_m);
  return {
    x_m: dx * right.x + dy * right.y,
    y_m: dx * forward.x + dy * forward.y,
  };
}

function sensorCanOwnLayer(radar) {
  if (radar.simulated) {
    return true;
  }
  if (!radar.node_id || !state.nodes[radar.node_id]) {
    return false;
  }
  return getNodeHealth(radar.node_id).online;
}

function isCoveredByHigherPrioritySensor(floor, sourceRadar, globalPoint) {
  const sourcePriority = Number(sourceRadar.priority || 0);
  return orderedRadars(floor).some((otherRadar) => {
    if (otherRadar.id === sourceRadar.id) {
      return false;
    }
    if (Number(otherRadar.priority || 0) <= sourcePriority) {
      return false;
    }
    if (!sensorCanOwnLayer(otherRadar)) {
      return false;
    }
    return isTargetVisibleForSensor(otherRadar, globalToSensorLocal(otherRadar, globalPoint));
  });
}

function getFloor(floorId) {
  return appConfig.floors.find((floor) => floor.id === floorId);
}

function getSelection() {
  const floor = getFloor(selectedRadar.floorId);
  if (!floor) return null;
  const radar = floor.radars.find((item) => item.id === selectedRadar.radarId);
  if (!radar) return null;
  return { floor, radar };
}

function setSelection(floorId = "", radarId = "") {
  selectedRadar = { floorId, radarId };
  renderFloorViewsOnly();
  renderInspector();
}

function persistLayout() {
  if (layoutSaveTimer) {
    window.clearTimeout(layoutSaveTimer);
  }
  layoutSaveTimer = window.setTimeout(() => {
    saveLayoutNow();
  }, LAYOUT_SAVE_DELAY_MS);
}

async function saveLayoutNow() {
  if (layoutSaveTimer) {
    window.clearTimeout(layoutSaveTimer);
    layoutSaveTimer = null;
  }

  try {
    await fetch("/api/layout", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        layout: appConfig,
        source_client_id: CLIENT_ID,
      }),
    });
  } catch (error) {
    console.warn("Failed to save shared layout", error);
  }
}

function selectionStillExists() {
  return Boolean(getSelection());
}

function applyLayout(nextLayout) {
  normalizeLayoutPriorities(nextLayout);
  appConfig = nextLayout;
  if (!appConfig.floors.some((floor) => floor.id === activeFloorId)) {
    activeFloorId = appConfig.floors[0]?.id || "";
  }
  if (!selectionStillExists()) {
    selectedRadar = { floorId: "", radarId: "" };
  }
  updateStatus();
  renderWorkspace();
  renderInspector();
}

function updateStatus() {
  const nodes = visibleNodes();
  const targets = nodes.reduce((sum, node) => sum + node.targets.length, 0);
  const healthList = nodes.map((node) => getNodeHealth(node.node_id));
  const onlineNodes = healthList.filter((health) => health.present && health.online);
  const freshest = healthList
    .filter((health) => health.present && health.ageMs !== null)
    .reduce((best, health) => (best === null || health.ageMs < best ? health.ageMs : best), null);

  nodeCountEl.textContent = String(nodes.length);
  onlineCountEl.textContent = String(onlineNodes.length);
  targetCountEl.textContent = String(targets);
  lastUpdateEl.textContent = state.last_update_ms
    ? new Date(state.last_update_ms).toLocaleTimeString()
    : "-";
  freshestAgeEl.textContent = formatAge(freshest);
  scaleLabelEl.textContent = `Default 1m = ${appConfig.global_scale.pixels_per_meter}px`;
}

function currentNodeSignature() {
  return visibleNodeEntries().map(([nodeId]) => nodeId).sort().join("|");
}

function currentLidarSignature() {
  return Object.keys(lidarState).sort().join("|");
}

function formatAge(ageMs) {
  if (ageMs === null || ageMs === undefined) return "-";
  if (ageMs < 1000) return `${Math.round(ageMs)} ms`;
  return `${(ageMs / 1000).toFixed(2)} s`;
}

function formatHz(hz) {
  if (!hz || hz <= 0) return "-";
  return `${hz.toFixed(1)} Hz`;
}

function getNodeHealth(nodeId, now = Date.now()) {
  const node = state.nodes[nodeId];
  if (!node) {
    return { present: false, online: false, ageMs: null, hz: 0, targetCount: 0 };
  }

  const receivedAtMs = node.received_at_ms ?? null;
  const ageMs = receivedAtMs ? Math.max(0, now - receivedAtMs) : null;
  return {
    present: true,
    nodeId,
    online: ageMs !== null && ageMs <= NODE_ONLINE_MAX_AGE_MS,
    ageMs,
    hz: Number(node.update_hz || 0),
    targetCount: Number(node.target_count ?? node.targets.length ?? 0),
    packetCount: Number(node.packet_count || 0),
    receivedAtMs,
  };
}

function nodeHealthCardMarkup(health) {
  if (!health.present) {
    return "No live data.";
  }

  return `
    <div class="node-health-head">
      <span class="node-health-id">${health.nodeId}</span>
      <span class="node-health-badge ${health.online ? "online" : "offline"}">
        ${health.online ? "online" : "offline"}
      </span>
    </div>
    <div class="node-health-grid">
      <div class="node-health-metric">
        <span class="node-health-label">Age</span>
        <span class="node-health-value">${formatAge(health.ageMs)}</span>
      </div>
      <div class="node-health-metric">
        <span class="node-health-label">Update</span>
        <span class="node-health-value">${formatHz(health.hz)}</span>
      </div>
      <div class="node-health-metric">
        <span class="node-health-label">Targets</span>
        <span class="node-health-value">${health.targetCount}</span>
      </div>
      <div class="node-health-metric">
        <span class="node-health-label">Packets</span>
        <span class="node-health-value">${health.packetCount}</span>
      </div>
    </div>
  `;
}

function renderNodeHealthList() {
  const nodeIds = visibleNodeEntries().map(([nodeId]) => nodeId).sort();
  if (nodeIds.length === 0) {
    nodeHealthList.innerHTML = '<div class="node-health-card empty">No devices seen yet.</div>';
    return;
  }

  nodeHealthList.innerHTML = nodeIds
    .map((nodeId) => `<article class="node-health-card">${nodeHealthCardMarkup(getNodeHealth(nodeId))}</article>`)
    .join("");
}

function getLidarHealth(nodeId, now = Date.now()) {
  const scan = lidarState[nodeId];
  if (!scan) {
    return { present: false, online: false, ageMs: null, pointCount: 0 };
  }

  const receivedAtMs = scan.received_at_ms ?? null;
  const ageMs = receivedAtMs ? Math.max(0, now - receivedAtMs) : null;
  return {
    present: true,
    online: ageMs !== null && ageMs <= LIDAR_ONLINE_MAX_AGE_MS,
    ageMs,
    pointCount: Number(scan.point_count || 0),
    validCount: Number(scan.valid_count || 0),
    speedHz: Number(scan.speed_hz || 0),
    scanRateHz: Number(scan.scan_rate_hz || 0),
    packetCount: Number(scan.packet_count || 0),
  };
}

function renderInspectorHealth() {
  const selection = getSelection();
  if (selection?.radar.simulated && !selection.radar.node_id) {
    inspectorNodeHealth.className = "node-health-card compact";
    inspectorNodeHealth.innerHTML = `
      <div class="node-health-head">
        <span class="node-health-id">demo_source</span>
        <span class="node-health-badge online">demo</span>
      </div>
      <div class="node-health-grid">
        <div class="node-health-metric">
          <span class="node-health-label">Mode</span>
          <span class="node-health-value">Simulated</span>
        </div>
        <div class="node-health-metric">
          <span class="node-health-label">Targets</span>
          <span class="node-health-value">2</span>
        </div>
      </div>
    `;
    return;
  }
  if (!selection || !selection.radar.node_id) {
    inspectorNodeHealth.className = "node-health-card compact empty";
    inspectorNodeHealth.textContent = "No device assigned.";
    return;
  }

  const lidarScan = getLidarScan(selection.radar.node_id);
  if (lidarScan && !state.nodes[selection.radar.node_id]) {
    const lidarHealth = getLidarHealth(selection.radar.node_id);
    inspectorNodeHealth.className = "node-health-card compact";
    inspectorNodeHealth.innerHTML = `
      <div class="node-health-head">
        <span class="node-health-id">${selection.radar.node_id}</span>
        <span class="node-health-badge ${lidarHealth.online ? "online" : "offline"}">
          ${lidarHealth.online ? "online" : "offline"}
        </span>
      </div>
      <div class="node-health-grid">
        <div class="node-health-metric">
          <span class="node-health-label">Age</span>
          <span class="node-health-value">${formatAge(lidarHealth.ageMs)}</span>
        </div>
        <div class="node-health-metric">
          <span class="node-health-label">Points</span>
          <span class="node-health-value">${lidarHealth.validCount}</span>
        </div>
        <div class="node-health-metric">
          <span class="node-health-label">Speed</span>
          <span class="node-health-value">${formatHz(lidarHealth.speedHz)}</span>
        </div>
        <div class="node-health-metric">
          <span class="node-health-label">Rate</span>
          <span class="node-health-value">${formatHz(lidarHealth.scanRateHz)}</span>
        </div>
      </div>
    `;
    return;
  }

  const health = getNodeHealth(selection.radar.node_id);
  if (!health.present) {
    inspectorNodeHealth.className = "node-health-card compact empty";
    inspectorNodeHealth.textContent = `Waiting for ${selection.radar.node_id}`;
    return;
  }

  inspectorNodeHealth.className = "node-health-card compact";
  inspectorNodeHealth.innerHTML = nodeHealthCardMarkup(health);
}

class FloorView {
  constructor(floor) {
    this.floor = floor;
    this.dragMode = null;
    this.dragRadarId = "";
    this.imageLoaded = false;
    this.hasImage = Boolean(floor.image_path);
    this.image = new Image();
    if (this.hasImage) {
      this.image.addEventListener("load", () => {
        this.imageLoaded = true;
        this.render();
      });
      this.image.src = `${floor.image_path}${floor.image_path.includes("?") ? "&" : "?"}v=${STATIC_VERSION}`;
    }
    this.build();
    this.attachEvents();
    this.render();
  }

  build() {
    this.root = document.createElement("section");
    this.root.className = "plan-card";
    if (this.floor.id === "floor_c") {
      this.root.classList.add("plan-card-wide");
    }
    this.root.id = `floor-card-${this.floor.id}`;
    const canvasWidth = Math.max(420, this.floor.image_width_px + 56);
    const canvasHeight = Math.max(300, this.floor.image_height_px + 56);
    this.root.innerHTML = `
        <div class="plan-card-header">
        <div>
          <h2>${this.floor.title}</h2>
          <p class="plan-copy">${this.hasImage
            ? "Click a sensor body to select it. Drag the center to move, drag the gold handle to rotate directional sensors."
            : (this.floor.placeholder_text || "Blank staging canvas.")}</p>
          <div class="floor-card-meta">
            <span>${planWidthMeters(this.floor).toFixed(2)}m wide</span>
            <span>${planHeightMeters(this.floor).toFixed(2)}m high</span>
          </div>
        </div>
        <div class="floor-card-actions">
          <button type="button" class="add-radar">Add Radar</button>
          <button type="button" class="add-lidar">Add Lidar</button>
        </div>
      </div>
      <div class="plan-stage">
        <canvas class="plan-canvas plan-static-canvas" width="${canvasWidth}" height="${canvasHeight}"></canvas>
        <canvas class="plan-canvas plan-overlay-canvas" width="${canvasWidth}" height="${canvasHeight}"></canvas>
      </div>
    `;
    floorplanGrid.appendChild(this.root);
    this.staticCanvas = this.root.querySelector(".plan-static-canvas");
    this.overlayCanvas = this.root.querySelector(".plan-overlay-canvas");
    this.canvas = this.overlayCanvas;
    this.staticCtx = this.staticCanvas.getContext("2d");
    this.overlayCtx = this.overlayCanvas.getContext("2d");
  }

  attachEvents() {
    this.root.querySelector(".add-radar").addEventListener("click", () => {
      const radar = createRadarTemplate(this.floor);
      this.floor.radars.push(radar);
      normalizeFloorPriorities(this.floor);
      persistLayout();
      setSelection(this.floor.id, radar.id);
    });

    this.root.querySelector(".add-lidar").addEventListener("click", () => {
      const lidar = createLidarTemplate(this.floor);
      this.floor.radars.push(lidar);
      normalizeFloorPriorities(this.floor);
      persistLayout();
      setSelection(this.floor.id, lidar.id);
    });

    this.canvas.addEventListener("pointerdown", (event) => {
      const point = this.pointerPoint(event);
      const hit = this.hitTest(point);
      if (!hit) {
        setSelection("", "");
        return;
      }
      event.preventDefault();
      setSelection(this.floor.id, hit.radar.id);
      this.dragMode = hit.mode;
      this.dragRadarId = hit.radar.id;
      this.canvas.setPointerCapture(event.pointerId);
    });

    this.canvas.addEventListener("pointermove", (event) => {
      if (!this.dragMode || !this.dragRadarId) return;
      const radar = this.floor.radars.find((item) => item.id === this.dragRadarId);
      if (!radar) return;
      const point = this.pointerPoint(event);

      if (this.dragMode === "move") {
        const meters = this.canvasToMeters(point.x, point.y);
        radar.x_m = clamp(meters.x, 0, planWidthMeters(this.floor));
        radar.y_m = clamp(meters.y, 0, planHeightMeters(this.floor));
      } else {
        const center = this.metersToCanvas(radar.x_m, radar.y_m);
        const dx = point.x - center.x;
        const dy = center.y - point.y;
        radar.heading_deg = (Math.atan2(dx, dy) * 180) / Math.PI;
        if (radar.heading_deg < 0) radar.heading_deg += 360;
      }

      persistLayout();
      renderFloorViewsOnly();
      renderInspector();
    });

    const stopDragging = (event) => {
      if (event?.pointerId !== undefined && this.canvas.hasPointerCapture(event.pointerId)) {
        this.canvas.releasePointerCapture(event.pointerId);
      }
      this.dragMode = null;
      this.dragRadarId = "";
    };

    this.canvas.addEventListener("pointerup", stopDragging);
    this.canvas.addEventListener("pointercancel", stopDragging);
  }

  pointerPoint(event) {
    const rect = this.canvas.getBoundingClientRect();
    const scaleX = this.canvas.width / rect.width;
    const scaleY = this.canvas.height / rect.height;
    return {
      x: (event.clientX - rect.left) * scaleX,
      y: (event.clientY - rect.top) * scaleY,
    };
  }

  getLayout() {
    const margin = 22;
    const availableWidth = this.canvas.width - margin * 2;
    const availableHeight = this.canvas.height - margin * 2;
    const scale = Math.min(
      availableWidth / this.floor.image_width_px,
      availableHeight / this.floor.image_height_px
    );
    const drawWidth = this.floor.image_width_px * scale;
    const drawHeight = this.floor.image_height_px * scale;
    const offsetX = (this.canvas.width - drawWidth) / 2;
    const offsetY = (this.canvas.height - drawHeight) / 2;
    return { scale, drawWidth, drawHeight, offsetX, offsetY };
  }

  metersToCanvas(xM, yM) {
      const { scale, offsetX, offsetY } = this.getLayout();
    const pixelsPerMeter = floorPixelsPerMeter(this.floor);
    const imageX = xM * pixelsPerMeter;
    const imageY = this.floor.image_height_px - yM * pixelsPerMeter;
      return { x: offsetX + imageX * scale, y: offsetY + imageY * scale };
    }

    canvasToMeters(x, y) {
      const { scale, offsetX, offsetY } = this.getLayout();
    const pixelsPerMeter = floorPixelsPerMeter(this.floor);
      const imageX = clamp((x - offsetX) / scale, 0, this.floor.image_width_px);
      const imageY = clamp((y - offsetY) / scale, 0, this.floor.image_height_px);
      return {
      x: imageX / pixelsPerMeter,
      y: (this.floor.image_height_px - imageY) / pixelsPerMeter,
      };
    }

  mapTarget(radar, target) {
    const { forward, right } = headingVectors(radar.heading_deg);
    const localX = radar.mirror_x ? -target.x_m : target.x_m;
    const localY = target.y_m;
    return {
      x: radar.x_m + localX * right.x + localY * forward.x,
      y: radar.y_m + localX * right.y + localY * forward.y,
    };
  }

  rotateHandle(radar) {
    if (isOmniSensor(radar)) {
      return null;
    }
    const center = this.metersToCanvas(radar.x_m, radar.y_m);
    const { forward } = headingVectors(radar.heading_deg);
    return { x: center.x + forward.x * 42, y: center.y - forward.y * 42 };
  }

  hitTest(point) {
    let best = null;
    [...orderedRadars(this.floor)].reverse().forEach((radar) => {
      const center = this.metersToCanvas(radar.x_m, radar.y_m);
      const handle = this.rotateHandle(radar);
      if (handle && Math.hypot(point.x - handle.x, point.y - handle.y) <= 12) {
        best = { radar, mode: "rotate" };
        return;
      }
      if (Math.hypot(point.x - center.x, point.y - center.y) <= 18) {
        best = { radar, mode: "move" };
      }
    });
    return best;
  }

  drawBackground() {
    const { drawWidth, drawHeight, offsetX, offsetY } = this.getLayout();
    this.staticCtx.clearRect(0, 0, this.staticCanvas.width, this.staticCanvas.height);
    if (this.imageLoaded) {
      this.staticCtx.drawImage(this.image, offsetX, offsetY, drawWidth, drawHeight);
    } else {
      this.staticCtx.fillStyle = this.hasImage ? "#ffffff" : "#f5f1e7";
      this.staticCtx.fillRect(offsetX, offsetY, drawWidth, drawHeight);
      if (!this.hasImage) {
        this.staticCtx.save();
        this.staticCtx.strokeStyle = "rgba(20, 35, 29, 0.15)";
        this.staticCtx.lineWidth = 2;
        this.staticCtx.setLineDash([10, 10]);
        this.staticCtx.strokeRect(offsetX + 8, offsetY + 8, drawWidth - 16, drawHeight - 16);
        this.staticCtx.setLineDash([]);
        this.staticCtx.fillStyle = "rgba(20, 35, 29, 0.7)";
        this.staticCtx.font = "600 24px Segoe UI";
        this.staticCtx.textAlign = "center";
        this.staticCtx.fillText("Floor C Placeholder", offsetX + drawWidth / 2, offsetY + drawHeight / 2 - 8);
        this.staticCtx.font = "15px Segoe UI";
        this.staticCtx.fillStyle = "rgba(20, 35, 29, 0.52)";
        this.staticCtx.fillText(
          this.floor.placeholder_text || "Blank staging canvas.",
          offsetX + drawWidth / 2,
          offsetY + drawHeight / 2 + 24
        );
        this.staticCtx.restore();
      }
    }
    this.staticCtx.fillStyle = palette.imageOverlay;
    this.staticCtx.fillRect(offsetX, offsetY, drawWidth, drawHeight);
  }

  drawTargets() {
      this.overlayCtx.clearRect(0, 0, this.overlayCanvas.width, this.overlayCanvas.height);
      orderedRadars(this.floor).forEach((radar) => {
        if (isHiddenNodeId(radar.node_id)) {
          return;
        }
        const lidarScan = getLidarScan(radar.node_id);
        if (lidarScan) {
          this.drawLidarScan(radar, lidarScan);
          return;
        }
        const targets = radar.simulated
          ? generateSimulatedTargets(radar)
          : (!radar.node_id || !state.nodes[radar.node_id] ? [] : state.nodes[radar.node_id].targets);

      targets.forEach((target) => {
        if (!isTargetVisibleForSensor(radar, target)) {
          return;
        }
        const mapped = this.mapTarget(radar, target);
        if (isCoveredByHigherPrioritySensor(this.floor, radar, mapped)) {
          return;
        }
        const center = this.metersToCanvas(radar.x_m, radar.y_m);
        const point = this.metersToCanvas(mapped.x, mapped.y);

        this.overlayCtx.save();
        this.overlayCtx.strokeStyle = palette.targetLine;
        this.overlayCtx.lineWidth = 2;
        this.overlayCtx.setLineDash([6, 6]);
        this.overlayCtx.beginPath();
        this.overlayCtx.moveTo(center.x, center.y);
        this.overlayCtx.lineTo(point.x, point.y);
        this.overlayCtx.stroke();
        this.overlayCtx.setLineDash([]);

        this.overlayCtx.fillStyle = palette.targetHalo;
        this.overlayCtx.beginPath();
        this.overlayCtx.arc(point.x, point.y, 17, 0, Math.PI * 2);
        this.overlayCtx.fill();

        this.overlayCtx.fillStyle = palette.target;
        this.overlayCtx.beginPath();
        this.overlayCtx.arc(point.x, point.y, 8, 0, Math.PI * 2);
        this.overlayCtx.fill();
        this.overlayCtx.restore();
      });
      });
    }

  drawLidarScan(radar, lidarScan) {
      lidarScan.points.forEach((point) => {
        if (!isTargetVisibleForSensor(radar, point)) {
          return;
        }
        const mapped = this.mapTarget(radar, point);
        const canvasPoint = this.metersToCanvas(mapped.x, mapped.y);
        const intensity = clamp(Number(point.intensity || 0), 0, 255) / 255;
        const hue = 220 - intensity * 160;
        const lightness = 48 + intensity * 28;
        this.overlayCtx.fillStyle = `hsla(${hue}deg 95% ${lightness}% / 0.92)`;
        this.overlayCtx.beginPath();
        this.overlayCtx.arc(canvasPoint.x, canvasPoint.y, intensity > 0.7 ? 2.6 : 1.9, 0, Math.PI * 2);
        this.overlayCtx.fill();
      });
    }

  drawRadar(radar) {
    const center = this.metersToCanvas(radar.x_m, radar.y_m);
      const { forward } = headingVectors(radar.heading_deg);
      const selected = selectedRadar.floorId === this.floor.id && selectedRadar.radarId === radar.id;
    const radiusPx = radar.range_m * floorPixelsPerMeter(this.floor) * this.getLayout().scale;

    this.staticCtx.save();
    this.staticCtx.fillStyle = selected ? "rgba(14, 143, 115, 0.18)" : palette.nodeCone;
    this.staticCtx.strokeStyle = selected ? palette.nodeSelected : radar.color;
    this.staticCtx.lineWidth = selected ? 3 : 2;

    if (isOmniSensor(radar)) {
      this.staticCtx.beginPath();
      this.staticCtx.arc(center.x, center.y, radiusPx, 0, Math.PI * 2);
      this.staticCtx.fill();
      this.staticCtx.stroke();
    } else {
      const sectorPoints = [center];
      for (let angle = -radar.fov_deg / 2; angle <= radar.fov_deg / 2; angle += 3) {
        const rotated = rotateVector(forward, (angle * Math.PI) / 180);
        const world = {
          x: radar.x_m + rotated.x * radar.range_m,
          y: radar.y_m + rotated.y * radar.range_m,
        };
        sectorPoints.push(this.metersToCanvas(world.x, world.y));
      }
      this.staticCtx.beginPath();
      this.staticCtx.moveTo(sectorPoints[0].x, sectorPoints[0].y);
      for (let i = 1; i < sectorPoints.length; i += 1) {
        this.staticCtx.lineTo(sectorPoints[i].x, sectorPoints[i].y);
      }
      this.staticCtx.closePath();
      this.staticCtx.fill();
      this.staticCtx.stroke();
    }

    this.staticCtx.fillStyle = selected ? palette.nodeSelected : radar.color;
    this.staticCtx.beginPath();
    this.staticCtx.arc(center.x, center.y, selected ? 15 : 13, 0, Math.PI * 2);
    this.staticCtx.fill();

    const handle = this.rotateHandle(radar);
    if (handle) {
      this.staticCtx.strokeStyle = selected ? palette.nodeSelected : radar.color;
      this.staticCtx.lineWidth = 2;
      this.staticCtx.beginPath();
      this.staticCtx.moveTo(center.x, center.y);
      this.staticCtx.lineTo(handle.x, handle.y);
      this.staticCtx.stroke();

      this.staticCtx.fillStyle = palette.handle;
      this.staticCtx.beginPath();
      this.staticCtx.arc(handle.x, handle.y, 8, 0, Math.PI * 2);
      this.staticCtx.fill();
    } else {
      this.staticCtx.strokeStyle = selected ? palette.nodeSelected : radar.color;
      this.staticCtx.lineWidth = 2;
      this.staticCtx.beginPath();
      this.staticCtx.arc(center.x, center.y, 24, 0, Math.PI * 2);
      this.staticCtx.stroke();
    }

    this.staticCtx.fillStyle = palette.text;
    this.staticCtx.font = "12px Segoe UI";
    this.staticCtx.fillText(radar.label, center.x + 16, center.y - 16);
    this.staticCtx.restore();
  }

  renderStatic() {
    this.drawBackground();
    orderedRadars(this.floor).forEach((radar) => this.drawRadar(radar));
  }

  renderTargetsOnly() {
    this.drawTargets();
  }

  render() {
    this.renderStatic();
    this.renderTargetsOnly();
  }
}

class LidarView {
  constructor(nodeId) {
    this.nodeId = nodeId;
    this.scan = null;
    this.build();
  }

  build() {
    this.root = document.createElement("article");
    this.root.className = "lidar-card";
    this.root.innerHTML = `
      <div class="lidar-card-head">
        <div>
          <h3>${this.nodeId}</h3>
          <p class="lidar-card-copy">Viewer-style raw point cloud from LD19 packets.</p>
        </div>
        <span class="lidar-badge offline">Offline</span>
      </div>
      <div class="lidar-card-meta">
        <div class="lidar-metric">
          <span class="lidar-metric-label">Speed</span>
          <strong class="lidar-metric-value" data-field="speed">-</strong>
        </div>
        <div class="lidar-metric">
          <span class="lidar-metric-label">Rate</span>
          <strong class="lidar-metric-value" data-field="rate">-</strong>
        </div>
        <div class="lidar-metric">
          <span class="lidar-metric-label">Valid</span>
          <strong class="lidar-metric-value" data-field="valid">-</strong>
        </div>
        <div class="lidar-metric">
          <span class="lidar-metric-label">Age</span>
          <strong class="lidar-metric-value" data-field="age">-</strong>
        </div>
      </div>
      <div class="lidar-canvas-shell">
        <canvas class="lidar-canvas" width="760" height="760"></canvas>
      </div>
      <div class="lidar-card-meta lidar-pipeline-meta">
        <div class="lidar-metric">
          <span class="lidar-metric-label">BG + KF</span>
          <strong class="lidar-metric-value" data-field="motion-count">0</strong>
        </div>
      </div>
    `;
    this.badgeEl = this.root.querySelector(".lidar-badge");
    this.speedEl = this.root.querySelector('[data-field="speed"]');
    this.rateEl = this.root.querySelector('[data-field="rate"]');
    this.validEl = this.root.querySelector('[data-field="valid"]');
    this.ageEl = this.root.querySelector('[data-field="age"]');
    this.motionCountEl = this.root.querySelector('[data-field="motion-count"]');
    this.canvas = this.root.querySelector(".lidar-canvas");
    this.ctx = this.canvas.getContext("2d");
    lidarViewerList.appendChild(this.root);
  }

  pointColor(intensity) {
    const normalized = clamp(Number(intensity || 0), 0, 255) / 255;
    const hue = 220 - normalized * 160;
    const lightness = 48 + normalized * 28;
    return `hsl(${hue}deg 95% ${lightness}%)`;
  }

  pointToCanvas(point) {
    const padding = 44;
    const radiusPx = (this.canvas.width / 2) - padding;
    const scale = radiusPx / LIDAR_VIEW_RANGE_M;
    return {
      x: this.canvas.width / 2 + Number(point.x_m || 0) * scale,
      y: this.canvas.height / 2 - Number(point.y_m || 0) * scale,
    };
  }

  drawGrid() {
    const centerX = this.canvas.width / 2;
    const centerY = this.canvas.height / 2;
    const radiusPx = (this.canvas.width / 2) - 44;
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    this.ctx.fillStyle = "#040708";
    this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

    this.ctx.save();
    this.ctx.strokeStyle = "rgba(166, 255, 214, 0.10)";
    this.ctx.lineWidth = 1;
    for (let ring = 1; ring <= 6; ring += 1) {
      this.ctx.beginPath();
      this.ctx.arc(centerX, centerY, (radiusPx / 6) * ring, 0, Math.PI * 2);
      this.ctx.stroke();
    }

    for (let angle = 0; angle < 360; angle += 30) {
      const rad = (angle * Math.PI) / 180;
      const x = centerX + Math.sin(rad) * radiusPx;
      const y = centerY - Math.cos(rad) * radiusPx;
      this.ctx.beginPath();
      this.ctx.moveTo(centerX, centerY);
      this.ctx.lineTo(x, y);
      this.ctx.stroke();
    }

    this.ctx.fillStyle = "rgba(184, 255, 223, 0.55)";
    this.ctx.font = "600 22px Segoe UI";
    this.ctx.textAlign = "center";
    this.ctx.fillText("0", centerX, 26);
    this.ctx.fillText("90", this.canvas.width - 26, centerY + 8);
    this.ctx.fillText("180", centerX, this.canvas.height - 16);
    this.ctx.fillText("270", 28, centerY + 8);

    this.ctx.font = "13px Segoe UI";
    for (let meter = 2; meter <= 12; meter += 2) {
      const y = centerY - (radiusPx / 12) * meter;
      this.ctx.fillText(`${meter}m`, centerX + 24, y - 2);
    }
    this.ctx.restore();
  }

  drawPoints() {
    if (!this.scan) {
      return;
    }
    this.scan.points.forEach((point) => {
      const mapped = this.pointToCanvas(point);
      this.ctx.fillStyle = this.pointColor(point.intensity);
      this.ctx.beginPath();
      this.ctx.arc(mapped.x, mapped.y, point.intensity > 180 ? 2.8 : 2.1, 0, Math.PI * 2);
      this.ctx.fill();
    });

    this.ctx.save();
    this.ctx.fillStyle = "rgba(90, 255, 182, 0.88)";
    this.ctx.beginPath();
    this.ctx.arc(this.canvas.width / 2, this.canvas.height / 2, 4.5, 0, Math.PI * 2);
    this.ctx.fill();
    this.ctx.restore();
  }

  pipelineTargets(suffix) {
    return state.nodes[`${this.nodeId}${suffix}`]?.targets || [];
  }

  drawDetections() {
    const motionTargets = this.pipelineTargets("__motion");

    motionTargets.forEach((target) => {
      const mapped = this.pointToCanvas(target);
      this.ctx.save();
      this.ctx.strokeStyle = "#5dffb3";
      this.ctx.lineWidth = 3;
      this.ctx.beginPath();
      this.ctx.arc(mapped.x, mapped.y, 12, 0, Math.PI * 2);
      this.ctx.stroke();
      this.ctx.fillStyle = "#5dffb3";
      this.ctx.font = "600 14px Segoe UI";
      this.ctx.fillText(`M${target.target_id}`, mapped.x + 14, mapped.y - 8);
      this.ctx.restore();
    });
  }

  update(scan) {
    this.scan = scan;
    const health = getLidarHealth(this.nodeId);
    this.badgeEl.className = `lidar-badge ${health.online ? "online" : "offline"}`;
    this.badgeEl.textContent = health.online ? "Online" : "Offline";
    this.speedEl.textContent = scan ? `${Number(scan.speed_hz || 0).toFixed(1)}` : "-";
    this.rateEl.textContent = scan ? `${Number(scan.scan_rate_hz || 0).toFixed(1)}` : "-";
    this.validEl.textContent = scan ? `${scan.valid_count}/${scan.point_count}` : "-";
    this.ageEl.textContent = health.ageMs === null ? "-" : `${Math.round(health.ageMs)} ms`;
    this.motionCountEl.textContent = String(this.pipelineTargets("__motion").length);
    this.drawGrid();
    this.drawPoints();
    this.drawDetections();
  }
}

function syncLidarViews() {
  const nodeIds = Object.keys(lidarState).sort();
  if (nodeIds.length === 0) {
    lidarViewerList.innerHTML = '<div class="lidar-empty">No LiDAR scans yet. Start the LD19 reader and a viewer panel will appear here.</div>';
    lidarViews.clear();
    return;
  }

  if (lidarViewerList.querySelector(".lidar-empty")) {
    lidarViewerList.innerHTML = "";
  }

  const liveSet = new Set(nodeIds);
  [...lidarViews.keys()].forEach((nodeId) => {
    if (liveSet.has(nodeId)) {
      return;
    }
    const view = lidarViews.get(nodeId);
    view?.root.remove();
    lidarViews.delete(nodeId);
  });

  nodeIds.forEach((nodeId) => {
    if (!lidarViews.has(nodeId)) {
      lidarViews.set(nodeId, new LidarView(nodeId));
    }
    lidarViews.get(nodeId).update(lidarState[nodeId]);
  });
}

function renderWorkspace() {
  floorplanGrid.innerHTML = "";
  floorViews.clear();
  appConfig.floors.forEach((floor) => {
    const view = new FloorView(floor);
    floorViews.set(floor.id, view);
  });
}

function buildNodeOptions(selectedNodeId) {
  const liveNodes = [...new Set([
    ...visibleNodeEntries().map(([nodeId]) => nodeId),
    ...visibleLidarNodeIds(),
  ])].sort();
  const options = ['<option value="">Unassigned</option>'];
  liveNodes.forEach((nodeId) => {
    options.push(`<option value="${nodeId}">${nodeId}</option>`);
  });
  inspectorNode.innerHTML = options.join("");
  inspectorNode.value = selectedNodeId || "";
}

function setInspectorDisabled(disabled) {
  [
    inspectorLabel,
    inspectorNode,
    inspectorX,
    inspectorY,
    inspectorHeadingRange,
    inspectorHeading,
    inspectorRange,
    inspectorFov,
    inspectorMirror,
    inspectorPriority,
    bringToFrontButton,
    sendToBackButton,
    duplicateRadarButton,
    removeRadarButton,
  ].forEach((element) => {
    element.disabled = disabled;
  });
}

function renderInspector() {
  const selection = getSelection();
  if (!selection) {
    inspectorTitle.textContent = "None Selected";
    inspectorSubtitle.textContent = "Click a sensor on any floor to edit it.";
    inspectorPreview.textContent = "{}";
    buildNodeOptions("");
    renderInspectorHealth();
    setInspectorDisabled(true);
    return;
  }

  const { floor, radar } = selection;
  inspectorTitle.textContent = radar.label;
  inspectorSubtitle.textContent = `${floor.title} | ${radar.id}`;
  inspectorLabel.value = radar.label;
  inspectorX.value = radar.x_m.toFixed(2);
  inspectorY.value = radar.y_m.toFixed(2);
  inspectorHeadingRange.value = String(Math.round(radar.heading_deg));
  inspectorHeading.value = String(Math.round(radar.heading_deg));
  inspectorRange.value = radar.range_m.toFixed(1);
  inspectorRange.max = isOmniSensor(radar) ? "15" : "10";
  inspectorRangeValue.textContent = `${Number(radar.range_m || 0).toFixed(1)}m`;
  inspectorFov.value = String(Math.round(isOmniSensor(radar) ? 360 : radar.fov_deg));
  inspectorFovValue.textContent = `${Math.round(isOmniSensor(radar) ? 360 : radar.fov_deg)}°`;
  inspectorFovField.style.display = isOmniSensor(radar) ? "none" : "";
  inspectorMirror.checked = Boolean(radar.mirror_x);
  inspectorPriority.value = String(Number(radar.priority || 0));
  buildNodeOptions(radar.node_id);
  renderInspectorHealth();
  inspectorPreview.textContent = JSON.stringify({ floor_id: floor.id, ...radar }, null, 2);
  setInspectorDisabled(false);
}

function mutateSelection(mutator) {
  const selection = getSelection();
  if (!selection) return;
  mutator(selection.floor, selection.radar);
  persistLayout();
  renderFloorViewsOnly();
  renderInspector();
}

function wireInspector() {
  inspectorLabel.addEventListener("input", () => {
    mutateSelection((_, radar) => {
      radar.label = inspectorLabel.value || radar.label;
    });
  });
  inspectorNode.addEventListener("change", () => {
    mutateSelection((_, radar) => {
      radar.node_id = inspectorNode.value;
    });
  });
  inspectorX.addEventListener("input", () => {
    mutateSelection((floor, radar) => {
      radar.x_m = clamp(Number(inspectorX.value), 0, planWidthMeters(floor));
    });
  });
  inspectorY.addEventListener("input", () => {
    mutateSelection((floor, radar) => {
      radar.y_m = clamp(Number(inspectorY.value), 0, planHeightMeters(floor));
    });
  });

  const headingHandler = (value) => {
    mutateSelection((_, radar) => {
      radar.heading_deg = ((Number(value) % 360) + 360) % 360;
    });
  };
  inspectorHeadingRange.addEventListener("input", () => headingHandler(inspectorHeadingRange.value));
  inspectorHeading.addEventListener("input", () => headingHandler(inspectorHeading.value));

  inspectorRange.addEventListener("input", () => {
    mutateSelection((_, radar) => {
      radar.range_m = clamp(Number(inspectorRange.value), 0.5, isOmniSensor(radar) ? 15 : 10);
    });
  });
  inspectorFov.addEventListener("input", () => {
    mutateSelection((_, radar) => {
      radar.fov_deg = isOmniSensor(radar) ? 360 : clamp(Number(inspectorFov.value), 1, 120);
    });
  });
  inspectorMirror.addEventListener("change", () => {
    mutateSelection((_, radar) => {
      radar.mirror_x = inspectorMirror.checked;
    });
  });

  bringToFrontButton.addEventListener("click", () => {
    mutateSelection((floor, radar) => {
      radar.priority = maxPriority(floor) + 1;
      normalizeFloorPriorities(floor);
    });
  });

  sendToBackButton.addEventListener("click", () => {
    mutateSelection((floor, radar) => {
      const minPriority = floor.radars.length === 0
        ? 0
        : Math.min(...floor.radars.map((item) => Number(item.priority || 0)));
      radar.priority = minPriority - 1;
      normalizeFloorPriorities(floor);
    });
  });

  duplicateRadarButton.addEventListener("click", () => {
    const selection = getSelection();
    if (!selection) return;
    const copy = deepClone(selection.radar);
    copy.id = `radar_${selection.floor.id}_${Date.now()}`;
    copy.label = `${selection.radar.label} Copy`;
    copy.x_m += 0.25;
    copy.y_m += 0.25;
    copy.priority = maxPriority(selection.floor) + 1;
    selection.floor.radars.push(copy);
    normalizeFloorPriorities(selection.floor);
    persistLayout();
    setSelection(selection.floor.id, copy.id);
  });

  removeRadarButton.addEventListener("click", () => {
    const selection = getSelection();
    if (!selection) return;
    selection.floor.radars = selection.floor.radars.filter((radar) => radar.id !== selection.radar.id);
    normalizeFloorPriorities(selection.floor);
    persistLayout();
    setSelection("", "");
  });

  exportLayoutButton.addEventListener("click", async () => {
    const json = JSON.stringify(appConfig, null, 2);
    inspectorPreview.textContent = json;
    try {
      await navigator.clipboard.writeText(json);
    } catch (error) {
      console.warn("Clipboard unavailable", error);
    }
  });

  clearSelectionButton.addEventListener("click", () => {
    setSelection("", "");
  });
}

async function loadConfig() {
  const response = await fetch("/api/layout");
  appConfig = await response.json();
  normalizeLayoutPriorities(appConfig);
  activeFloorId = appConfig.floors[0]?.id || "";
}

async function fetchInitialState() {
  const response = await fetch("/api/state");
  Object.assign(state, await response.json());
  updateStatus();
  renderNodeHealthList();
}

async function fetchInitialLidarState() {
  const response = await fetch("/api/lidar");
  const payload = await response.json();
  Object.keys(lidarState).forEach((nodeId) => delete lidarState[nodeId]);
  Object.assign(lidarState, payload);
  syncLidarViews();
}

function connectWebSocket() {
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  const ws = new WebSocket(`${protocol}://${window.location.host}/ws`);

  ws.addEventListener("open", () => {
    serverStatusEl.textContent = "Connected";
  });

  ws.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "state") {
      Object.assign(state, message.payload);
      updateStatus();
      renderNodeHealthList();
      renderInspectorHealth();
      floorViews.forEach((view) => view.renderTargetsOnly());
      lidarViews.forEach((view, nodeId) => view.update(lidarState[nodeId]));
      const nextNodeSignature = currentNodeSignature();
      if (nextNodeSignature !== lastNodeSignature) {
        lastNodeSignature = nextNodeSignature;
        renderInspector();
      }
      return;
    }

    if (message.type === "layout") {
      if (message.source_client_id && message.source_client_id === CLIENT_ID) {
        return;
      }
      applyLayout(message.payload);
      return;
    }

    if (message.type === "lidar_state") {
      Object.keys(lidarState).forEach((nodeId) => delete lidarState[nodeId]);
      Object.assign(lidarState, message.payload);
      syncLidarViews();
      lastLidarSignature = currentLidarSignature();
      return;
    }

    if (message.type === "lidar_scan") {
      lidarState[message.payload.node_id] = message.payload;
      syncLidarViews();
      const nextLidarSignature = currentLidarSignature();
      if (nextLidarSignature !== lastLidarSignature) {
        lastLidarSignature = nextLidarSignature;
      }
    }
  });

  ws.addEventListener("close", () => {
    serverStatusEl.textContent = "Disconnected, retrying...";
    setTimeout(connectWebSocket, 1000);
  });

  ws.addEventListener("error", () => {
    serverStatusEl.textContent = "WebSocket error";
  });
}

async function init() {
  await loadConfig();
  await fetchInitialState();
  await fetchInitialLidarState();
  lastNodeSignature = currentNodeSignature();
  lastLidarSignature = currentLidarSignature();
  wireInspector();
  renderWorkspace();
  renderInspector();
  window.setInterval(() => {
    updateStatus();
    renderNodeHealthList();
    renderInspectorHealth();
    lidarViews.forEach((view, nodeId) => view.update(lidarState[nodeId]));
  }, HEALTH_TICK_MS);
  window.setInterval(() => {
    floorViews.forEach((view) => view.renderTargetsOnly());
  }, DEMO_TICK_MS);
  connectWebSocket();
}

init();
