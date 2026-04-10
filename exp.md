# é¡¹ç›®æè¿°
æœ¬å®žéªŒçš„ç›®æ ‡æ˜¯æž„å»ºä¸€ä¸ª**åŸºäºŽå…¬å¯“ 2D åº•å›¾çš„å®¤å†…å¤šé›·è¾¾äººä½“ä½ç½®æ˜¾ç¤ºç³»ç»Ÿ**ã€‚è¾“å…¥ä¸ºå…¬å¯“é¸Ÿçž°å¹³é¢å›¾å’Œè‹¥å¹²ä¸ª LD2450 é›·è¾¾èŠ‚ç‚¹ï¼Œè¾“å‡ºä¸ºæµè§ˆå™¨ä¸­çš„å®žæ—¶ 2D å¹³é¢å›¾ï¼Œèƒ½å¤Ÿåœ¨åº•å›¾ä¸Šæ˜¾ç¤ºå½“å‰å±‹å†…äººä½“ç›®æ ‡çš„å¤§è‡´ä½ç½®ã€‚ç³»ç»Ÿä»…å…³æ³¨**é›·è¾¾å±€éƒ¨åæ ‡åˆ°å…¬å¯“å…¨å±€ 2D åæ ‡çš„æ˜ å°„ä¸Žå›¾å±‚è¦†ç›–æ˜¾ç¤º**ï¼Œä¸æ¶‰åŠ ARã€UWBã€ç¬¬ä¸€äººç§°é€è§†æˆ–å®¤å†… GPSã€‚

ç³»ç»Ÿæž¶æž„åˆ†ä¸ºå››å±‚ï¼šåº•å›¾å±‚ã€é›·è¾¾èŠ‚ç‚¹å±‚ã€æœåŠ¡å™¨è§„åˆ™å±‚å’Œå‰ç«¯æ˜¾ç¤ºå±‚ã€‚åº•å›¾å±‚æä¾›å…¬å¯“å¹³é¢å›¾åŠæ¯”ä¾‹å°ºï¼Œå®šä¹‰ç»Ÿä¸€ä¸–ç•Œåæ ‡ç³»ï¼›é›·è¾¾èŠ‚ç‚¹å±‚ç”± LD2450 ä¸Ž ESP32 ç»„æˆï¼Œè´Ÿè´£é‡‡é›†ç›®æ ‡æ•°æ®å¹¶é€šè¿‡ Wi-Fi ä¸Šä¼ ï¼›æœåŠ¡å™¨å±‚è´Ÿè´£å±€éƒ¨åæ ‡åˆ°å…¨å±€åæ ‡çš„å˜æ¢ã€è¿‡æ»¤ã€å›¾å±‚ä¼˜å…ˆçº§è£å‰ªä¸ŽçŠ¶æ€ç®¡ç†ï¼›å‰ç«¯è´Ÿè´£ç»˜åˆ¶åº•å›¾ã€é›·è¾¾å®‰è£…ä½ç½®ã€æœå‘åŠå®žæ—¶ç›®æ ‡ç‚¹ã€‚æ•´ä½“é“¾è·¯ä¸ºï¼š

`LD2450 -> ESP32 -> Wi-Fi -> Python Server -> Browser 2D Map`

æœ¬å®žéªŒé‡‡ç”¨çš„æœ€å°ç¡¬ä»¶é…ç½®å¦‚ä¸‹ï¼š

* **æ¯«ç±³æ³¢æ¨¡å—**ï¼šHLK-LD2450
* **ä¸»æŽ§æ¿**ï¼šESP32-WROOM-32 Dev Board
* **èŠ‚ç‚¹æ•°é‡**ï¼šV1 å»ºè®®ä»Ž 1 ä¸ªèŠ‚ç‚¹èµ·æ­¥ï¼ŒéªŒè¯åŽæ‰©å±•è‡³ 2 ä¸ªèŠ‚ç‚¹
* **æœåŠ¡å™¨**ï¼šä¸€å°ç”µè„‘ï¼Œè´Ÿè´£è¿è¡Œ Python åŽç«¯ä¸Žå‰ç«¯é¡µé¢
* **ç½‘ç»œ**ï¼šåŒä¸€å±€åŸŸç½‘ Wi-Fi

å½“å‰å•èŠ‚ç‚¹æœ€å°å®žéªŒçš„æ¿çº§è¿žæŽ¥é…ç½®å¦‚ä¸‹ï¼Œä»…ä½¿ç”¨ 4 æ ¹çº¿ï¼š

* **LD2450 `5V` â†’ ESP32 `VIN`**
* **LD2450 `GND` â†’ ESP32 `GND`**
* **LD2450 `TX` â†’ ESP32 `RX2`**
* **LD2450 `RX` â†’ ESP32 `TX2`**

ESP32 ä¸Šå¯¹åº”å…³ç³»ä¸ºï¼š

* **`RX2 = GPIO16`**
* **`TX2 = GPIO17`**

ä¸²å£ä½¿ç”¨ **UART2**ï¼Œé…ç½®ä¸ºï¼š

* æ³¢ç‰¹çŽ‡ï¼š**256000**
* å…¸åž‹åˆå§‹åŒ–æ–¹å¼ï¼š`Serial2.begin(256000, SERIAL_8N1, 16, 17)`

LD2450 å…¶ä»–å¼•è„šå¦‚ `3.3V`ã€`PA9`ã€`DP`ã€`DM` åœ¨å½“å‰å®žéªŒé˜¶æ®µå‡ä¸ä½¿ç”¨ã€‚

è½¯ä»¶çŽ¯å¢ƒå»ºè®®ä¸ºï¼š

* **èŠ‚ç‚¹ç«¯**ï¼šPlatformIO + Arduino Framework
* **æœåŠ¡å™¨ç«¯**ï¼šPython
* **å‰ç«¯**ï¼šç®€å•ç½‘é¡µï¼Œå¯ä½¿ç”¨ Canvas/SVG å®žæ—¶ç»˜å›¾

                              
## Current Project Status And Operations

### 1. Current Project Structure

```text
ld2450_esp32/
  src/
    main.cpp
  server/
    app.py
    serial_bridge.py
    requirements.txt
  web/
    index.html
    app.js
    style.css
    assets/
      floorplan_a.jpg
      floorplan_b.jpg
  config/
    floorplans.json
    right_upper_bedroom.json
  scripts/
    ld2450_viewer.py
  platformio.ini
  exp.md
  run_server.bat
  run_bridge.bat
  run_viewer.bat
```

Key responsibilities:

- `src/main.cpp`
  ESP32 firmware entry. Reads LD2450 from UART2, parses target frames, and uploads the latest target snapshot over Wi-Fi.

- `server/app.py`
  FastAPI backend. Serves the webpage, receives radar data, stores shared layout config, and broadcasts state/layout over WebSocket.

- `server/serial_bridge.py`
  Old fallback path. Reads `PLOT` lines from `COM4` and forwards them to the backend through HTTP. No longer the default path.

- `web/index.html`, `web/app.js`, `web/style.css`
  Frontend workspace for floorplan display, radar placement, heading adjustment, device assignment, and live target rendering.

- `config/floorplans.json`
  Shared layout config. Stores global scale, floor images, and radar placement data.


### 2. Current Implementation Method

Current live data path:

```text
LD2450 -> ESP32(UART2) -> ESP32 Wi-Fi HTTP POST -> FastAPI -> WebSocket -> Browser
```

UART settings:

- `GPIO16` = `RX2`
- `GPIO17` = `TX2`
- `Serial2.begin(256000, SERIAL_8N1, 16, 17)`

Frame format used by firmware:

- Header: `AA FF 03 00`
- Payload: `24 bytes`
- Footer: `55 CC`

Each 24-byte payload is decoded into up to 3 targets:

- `X`
- `Y`
- `Speed`
- `Gate`

Current backend endpoints:

- `GET /`
- `GET /api/state`
- `GET /api/layout`
- `PUT /api/layout`
- `POST /api/radar/{node_id}`
- `WS /ws`

Current frontend supports:

- Floor A / Floor B display
- Add / duplicate / remove radar
- Drag radar position
- Drag heading handle
- Assign `node_id`
- Shared layout across multiple devices on the LAN


### 3. Current Low-Latency Design

The first Wi-Fi version had noticeable delay because the ESP32 main loop did too much work:

- parse radar frames
- send synchronous HTTP POST
- print `PLOT` lines
- refresh large serial dashboard output

That blocked radar parsing and increased motion delay.

Current low-latency fixes in `src/main.cpp`:

1. The main loop only parses radar data.
2. HTTP upload runs in a separate FreeRTOS task.
3. Only the latest frame is kept in a queue of length `1`.
4. Wi-Fi sleep is disabled with `WiFi.setSleep(false)`.
5. `PLOT` stream is off by default.
6. Serial dashboard view is off by default.
7. Frontend drawing is split into static layer + target layer so only live target dots redraw frequently.


### 4. Current Run Flow

#### 4.1 Start backend server

```powershell
cd C:\Users\thorn\Downloads\Tentacleslab\PlatformIO\Projects\ld2450_esp32
python -m uvicorn server.app:app --host 0.0.0.0 --port 8000 --reload
```

Then open:

- local: `http://127.0.0.1:8000/`
- LAN: `http://<PC_LAN_IP>:8000/`

Example used during current testing:

- `http://192.168.1.128:8000/`


#### 4.2 Upload ESP32 firmware

```powershell
cd C:\Users\thorn\Downloads\Tentacleslab\PlatformIO\Projects\ld2450_esp32
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload --upload-port COM4
```


#### 4.3 Open webpage

Use any device on the same LAN:

- computer: `http://127.0.0.1:8000/`
- iPad / phone / another PC: `http://<PC_LAN_IP>:8000/`


### 5. Current Web Operation Method

#### 5.1 Select and move sensor

- click sensor body: select sensor
- drag sensor body: move `(x, y)`
- drag gold handle: adjust `heading`


#### 5.2 Heading definition

- `0Â°`: up
- `90Â°`: right
- `180Â°`: down
- `270Â°`: left


#### 5.3 Add and manage sensor

- use `Add Sensor` on a floor card
- after selecting a sensor, use inspector buttons:
  - `Bring To Front`
  - `Send To Back`
  - `Duplicate`
  - `Remove`


#### 5.4 Shared layout sync

Sensor placement is now stored server-side instead of only in browser local storage.

Shared layout file:

- `config/floorplans.json`

That means:

- computer changes can be seen on iPad after refresh
- all devices use the same base sensor placement config


#### 5.5 Current display limits

The browser only displays target dots that fall inside the selected sensor's current display sector:

- `Range (m)` maximum: `10`
- `Display Angle (deg)` maximum: `120`
- for a directional sensor, a target must be inside both:
  - range limit
  - display angle limit
- for the backyard `360 Sensor`, display is only limited by range

This is a front-end display rule. Raw node data can still reach the server even if the point is filtered out visually.


#### 5.6 Layer priority rule

- a newly added sensor defaults to the top layer
- `Bring To Front` raises a sensor to highest priority
- `Send To Back` lowers a sensor to lowest priority
- if a lower-priority sensor reports a point inside the visible sector of a higher-priority online sensor on the same floor, the lower-priority point is hidden
- this is display ownership, not fusion


#### 5.7 Current Floor C / Backyard state

- `Floor C / Backyard` now uses a real background image
- a centered demo `360 Sensor` is present
- the backyard demo sensor shows simulated moving points
- current demo sensor range is `12m`


### 6. Current Serial Debug Commands

If serial monitor is open, the firmware supports:

- `h` : print help
- `r` : toggle raw frame hex dump
- `p` : toggle `PLOT` serial stream
- `v` : toggle dashboard text view

For lowest delay, recommended default:

- keep `PLOT` off
- keep dashboard off


### 7. Recommended Standard Operation Order

1. Connect LD2450 to ESP32.
2. Connect ESP32 to the computer by USB.
3. Start the Python backend server.
4. Open the webpage.
5. Configure radar position and heading on the floorplan.
6. Walk inside the radar view area.
7. Observe live target dots in the browser.
8. Only enable serial debug commands if further troubleshooting is needed.


### 8. Current Completion Summary

Already completed:

- single-node LD2450 + ESP32 bring-up
- UART2 frame parsing
- Wi-Fi upload from ESP32
- FastAPI backend receiving radar data
- browser-based live floorplan visualization
- radar placement and heading configuration
- shared layout config across LAN devices
- one round of low-latency optimization
- live layer-priority controls in the inspector
- overlap suppression based on higher-priority sensor coverage
- backyard image area with demo `360 Sensor`

Still available for next phase:

- end-to-end latency display in the web UI
- multi-node access
- radar layer priority + coverage mask filtering
- trajectory smoothing
- wall / boundary constraints
- ESP32 AP setup mode + Preferences-based Wi-Fi/server config persistence
  This should be done late in the project, after the current radar pipeline, layout workflow, multi-node binding, and layer-priority workflow are stable.
  Recommended future provisioning flow:
  1. A new ESP32 boots into AP setup mode if no saved config exists.
  2. The device exposes a local config page such as `192.168.4.1`.
  3. The user enters `WiFi SSID`, `WiFi Password`, `Server URL`, and optional `node_id`.
  4. The ESP32 saves these settings into `Preferences` / NVS.
  5. The device reboots and joins the shared LAN automatically.
  6. The central web UI then discovers the node and only handles placement, heading, and device assignment.


### 9. LD2450 Target Fields In Current Project

Each decoded target currently includes:

- `x`
- `y`
- `speed`
- `gate`

Meaning in the current implementation:

- `x`: lateral position relative to the sensor
- `y`: forward distance relative to the sensor
- `speed`: target speed reported by LD2450
- `gate`: not target distance; it is closer to `distance resolution` / `range bin size`

Important note about `gate`:

- it should not be interpreted as the actual target distance
- it is better understood as the distance-direction resolution granularity used for that target report
- in future code/UI, this field should be renamed to something clearer such as `distance_resolution_mm` or `range_bin_mm`


### 10. FHL-LD19P / D300 Lidar Notes

Current reference page:

- `https://wiki.youyeetoo.com/en/Lidar/D300`

Important observation:

- the page path says `D300`, but the actual product content currently describes `FHL-LD19P Lidar`
- the page itself warns that some content may still be incomplete


#### 10.1 What has already been verified

- the lidar development kit is connected to the PC through the included USB adapter
- Windows software `LdsPointCloudViewer V3.0.6` is working
- current verified port: `COM4`
- current verified baudrate in the viewer: `230400`
- the viewer already shows live 360-degree scan data
- this confirms that:
  - the lidar hardware is working
  - the USB adapter is working
  - the serial link to the PC is working


#### 10.2 What the included adapter board is

Based on the product page and the adapter diagram:

- the included board is a `CP2102 USB to TTL` adapter
- its role is mainly:
  - convert the lidar serial interface into a PC-readable USB COM port
  - provide a convenient Windows evaluation path
- it should not be treated as a lidar protocol decoder

Important implication:

- the adapter board mainly bridges `TTL UART <-> USB`
- the actual protocol parsing is done by:
  - `LdsPointCloudViewer`
  - ROS/SDK code
  - or future custom code


#### 10.3 Why MCU / ESP32 direct connection is considered feasible

The official page explicitly states:

- `Or connect directly to the motherboard or MCU.`

This means:

- the product is intended to support direct MCU-style integration
- the included USB adapter is not mandatory in the final system
- in a future embedded design, the likely topology is:
  - `LiDAR -> ESP32 UART -> Wi-Fi -> server -> web`


#### 10.4 Current engineering conclusion

- for the current minimum implementation, keeping the lidar connected to the PC is the safest path
- this lidar is not like `LD2450`
- `LD2450` outputs already-processed target results such as `x / y / speed`
- `LD19P` is closer to a 360-degree scan source that must be further processed

Recommended minimum implementation path:

1. Keep using the included USB adapter and PC.
2. Read lidar data from `COM4`.
3. Parse scan data into angle/distance points.
4. Feed those points into the current `FastAPI + web` stack.
5. Render the 360-degree scan or point cloud on `Floor C / Backyard`.


#### 10.5 Future ESP32 integration note

If this lidar is later migrated to `ESP32`, the correct interpretation is:

- the `CP2102` adapter would normally be replaced by direct UART wiring to the ESP32
- the adapter is not the decoder; it is mainly the USB bridge
- the future validation sequence should be:
  1. confirm lidar power and UART pin mapping
  2. confirm the ESP32 can read the raw serial byte stream
  3. compare ESP32-side raw data with the current PC-side behavior
  4. only then move on to protocol parsing and Wi-Fi upload

Current caution:

- MCU support is strongly suggested by the official page
- however, final direct wiring to ESP32 should still be validated against the product's actual pinout, power requirement, and UART electrical level


### 11. Planned ESP32 + LiDAR Design

This section captures the currently agreed direction for the `LD19P / D300` lidar path.


#### 11.1 High-level goal

The long-term plan is not to treat the 2D lidar as a raw point-cloud-only device in the final user workflow.

Instead, the goal is:

- connect `LD19P` to `ESP32`
- process lidar scan data on the ESP32
- detect only moving objects
- output a unified target format:
  - `x`
  - `y`
  - `speed`
- send those results over Wi-Fi
- integrate them into the existing web-based sensor system alongside current `LD2450` nodes


#### 11.2 What this design is not trying to do

- no human identity recognition
- no semantic classification such as person / pet / object
- no 3D perception
- no full SLAM / mapping stack
- no high-level AI object labeling

Reason:

- this is a 2D lidar
- it is suitable for moving object tracking
- it is not suitable as the main sensor for reliable human identity recognition


#### 11.3 Unified output model

The lidar path is intended to end in the same style of output already used by the current radar web system:

- `x`: object position in local sensor coordinates
- `y`: object position in local sensor coordinates
- `speed`: object motion estimate

This is important because it allows:

- reuse of the current `sensor` abstraction in the web UI
- reuse of current placement / heading / range / display-angle controls
- reuse of current Wi-Fi upload and floor rendering ideas

In other words, the lidar pipeline is expected to convert:

- raw 360-degree scan points

into:

- target-level moving object results


#### 11.4 Planned processing stages on ESP32

The intended processing chain is:

1. `LD19P -> ESP32 UART`
2. decode scan packets into angle/distance samples
3. convert polar data into local `x/y` points
4. build a lightweight static background model
5. detect moving points by frame-to-frame or background difference
6. cluster moving points into object candidates
7. track clusters across frames
8. estimate per-object `speed`
9. upload unified target results over Wi-Fi


#### 11.5 Minimum viable version

The minimum viable ESP32 + lidar implementation should stay narrow:

- no human recognition
- no complex multi-object semantics
- no outdoor scene understanding
- just:
  - moving point filtering
  - simple clustering
  - simple object center extraction
  - simple speed estimate

Minimum output target:

- one or more moving object centroids
- each with `x / y / speed`


#### 11.6 Recommended milestone order

Recommended development order:

1. prove `ESP32` can read the raw lidar UART stream
2. decode scan packets
3. convert scan packets into local `x/y` points
4. render/debug those points
5. add moving-point extraction
6. add clustering
7. add cross-frame tracking
8. add `speed`
9. expose the result in the same format used by the current web system


#### 11.7 Current architecture implication

If this design succeeds, the lidar node will stop being a raw point-cloud-only node from the perspective of the main web app.

Instead, it will behave more like:

- a processed target sensor node

That means:

- the existing floor UI does not need to become a full lidar point-cloud workstation
- the main browser UI can continue to think in terms of:
  - sensors
  - placement
  - heading
  - displayed targets

The lidar-specific heavy lifting is planned to happen inside the ESP32 pipeline.


### 12. LD19 PC Demo Multi-Target Notes

Current `LD19` PC-side demo already supports multi-target detection at the algorithm level.

The implemented `M` pipeline is:

- raw `LD19` scan parsing on PC
- background subtraction
- clustering of moving foreground points
- shape-gate filtering
- multi-target tracking
- per-track `x / y / speed` output

As long as moving objects are spatially separated enough, they can appear as multiple targets.

Current practical limits:

- nearby moving objects can still merge into one cluster
- very small or very slow targets may be filtered out by the current thresholds
- baseline output still exists in the backend for reference, but is hidden in the web UI


### 13. Remote PC2 LiDAR Node Over Wi-Fi

It is valid to run:

- `LiDAR + original USB adapter + PC2`

and let that second PC send processed results over Wi-Fi to the current main server / web app.

Recommended topology:

- `PC1`:
  - runs `FastAPI`
  - hosts the main web UI
  - receives LiDAR target output from remote PCs

- `PC2`:
  - connects to `LD19P` over USB serial
  - runs `server/ld19_reader.py`
  - uploads:
    - raw scan stream to `/api/lidar/<node_id>`
    - moving-target results to `/api/radar/<node_id>__motion`


#### 13.1 Main server on PC1

Start the existing server on the main machine:

```powershell
cd C:\Users\thorn\Downloads\Tentacleslab\PlatformIO\Projects\ld2450_esp32
python -m uvicorn server.app:app --host 0.0.0.0 --port 8000 --reload
```

Find the LAN IP of `PC1`, for example:

```powershell
ipconfig
```

Then note the IPv4 address, such as:

- `192.168.1.128`

That address becomes the upload target for `PC2`.


#### 13.2 Reader on PC2

On the second machine:

```powershell
cd C:\Users\thorn\Downloads\Tentacleslab\PlatformIO\Projects\ld2450_esp32
pip install -r server\requirements.txt
python server\ld19_reader.py COM4 --server-url http://192.168.1.128:8000 --node-id ld19_backyard_pc2
```

Replace:

- `COM4` with the actual LiDAR serial port on `PC2`
- `192.168.1.128` with the actual IP of `PC1`
- `ld19_backyard_pc2` with the chosen node name for that LiDAR station


#### 13.3 What PC2 uploads

The reader uploads three logical streams:

- raw LiDAR scan node:
  - `ld19_backyard_pc2`

- main motion pipeline:
  - `ld19_backyard_pc2__motion`

- baseline pipeline:
  - `ld19_backyard_pc2__baseline`

The web UI currently hides the baseline node from normal display.


#### 13.4 How to use in the current web UI

After `PC2` starts uploading:

- the raw scan appears in the `LD19 Viewer / Point Cloud` section
- the motion-output node can be bound to a floor sensor using:
  - `ld19_backyard_pc2__motion`

This means:

- one machine can host the server and webpage
- another machine can host the LiDAR and algorithm
- the result still integrates into the current sensor map UI


#### 13.5 If PC2 is not on the same LAN

Use `Tailscale` between `PC1` and `PC2`.

Then `PC2` can upload to the Tailscale address of `PC1`, for example:

```powershell
python server\ld19_reader.py COM4 --server-url http://100.x.x.x:8000 --node-id ld19_backyard_pc2
```

In that case:

- `PC1` still runs the server
- `PC2` still runs the LiDAR reader
- communication goes through the Tailscale private network
