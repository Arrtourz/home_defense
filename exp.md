# 项目说明

## 1. 目标

本项目是一个基于 2D 底图的多传感器位置显示系统。

当前包含两条主线：

- `LD2450 + ESP32 + Wi‑Fi`
  输出目标级结果：`x / y / speed`
- `LD19P + PC`
  输出原始点云，并在 PC 侧做移动物体检测

最终目标是在浏览器中统一显示：

- 室内多雷达目标点
- 后院 LiDAR 点云与移动目标

不做：

- 人员身份识别
- AR / UWB / 室内 GPS
- 复杂融合


## 2. 当前目录结构

```text
ld2450_esp32/
  src/
    main.cpp
  server/
    app.py
    serial_bridge.py
    ld19_reader.py
    ld19_shape_gate_plugin.py
    requirements.txt
  web/
    index.html
    app.js
    style.css
    assets/
      floorplan_a.jpg
      floorplan_b.jpg
      backyard.png
      yard.png
      yard_766.png
  config/
    floorplans.json
    right_upper_bedroom.json
  scripts/
    ld2450_viewer.py
  platformio.ini
  exp.md
```


## 3. 当前实现

### 3.1 LD2450 室内链路

链路：

```text
LD2450 -> ESP32(UART2) -> Wi‑Fi HTTP POST -> FastAPI -> WebSocket -> Browser
```

硬件连接：

- `LD2450 5V -> ESP32 VIN`
- `LD2450 GND -> ESP32 GND`
- `LD2450 TX -> ESP32 RX2(GPIO16)`
- `LD2450 RX -> ESP32 TX2(GPIO17)`

UART 配置：

- `Serial2.begin(256000, SERIAL_8N1, 16, 17)`

当前网页能力：

- 多楼层显示
- 传感器位置拖拽
- heading 调整
- 图层优先级
- 设备绑定
- 局域网共享布局


### 3.2 LD19P 后院链路

链路：

```text
LD19P -> 原装 USB/CP2102 板 -> PC 串口 -> ld19_reader.py -> FastAPI -> Browser
```

已验证：

- Windows 工具 `LdsPointCloudViewer` 正常
- 当前示例串口：`COM4`
- 当前示例波特率：`230400`

网页中当前包含两种显示：

- `LD19 Viewer / Point Cloud`
  黑底极坐标点云视图
- `Floor C / Backyard`
  后院底图上的 LiDAR / 目标可视化


## 4. 当前网页规则

### 4.1 传感器类型

网页现在区分两类传感器：

- `Radar`
  - 定向
  - 有 `Display Angle`
  - `Range` 上限 `10m`
- `Lidar`
  - 全向
  - 固定 `360°`
  - `Range` 上限 `15m`

卡片按钮：

- `Add Radar`
- `Add Lidar`


### 4.2 图层规则

每个传感器都有优先级。

当前规则：

- 新增传感器默认在最上层
- 可用按钮：
  - `Bring To Front`
  - `Send To Back`
  - `Duplicate`
  - `Remove`
- 若低优先级传感器的点落在高优先级在线传感器的显示区域内，则低层点隐藏

这是图层覆盖，不是 fusion。


### 4.3 Floor C / Backyard

当前 `Floor C / Backyard` 使用：

- `web/assets/yard_766.png`

比例假设：

- `123 px = 5 m`

因此当前图像 `766 x 766 px` 对应的真实范围约为：

- `31.14 m x 31.14 m`


## 5. LD2450 说明

当前解析的目标字段：

- `x`
- `y`
- `speed`
- `gate`

说明：

- `x`：横向位置
- `y`：前向距离
- `speed`：模块输出的速度
- `gate`：更接近距离方向的分辨粒度，不是目标实际距离

后续如果继续整理命名，建议把 `gate` 改为：

- `range_bin_mm`
或
- `distance_resolution_mm`


## 6. LD19P 当前算法

### 6.1 原始点云

`ld19_reader.py` 已实现：

- 串口读取
- 协议解包
- `CRC8`
- 角度插值
- `angle / distance / intensity / x / y`


### 6.2 当前主算法 M

当前主检测链路：

- 点过滤
- 背景减除
- 前景聚类
- 形态门槛过滤
- 多目标跟踪
- 输出 `x / y / speed`

当前 `M` 是主显示算法。


### 6.3 形态门槛插件

独立模块：

- `server/ld19_shape_gate_plugin.py`

当前用于抑制背景干扰，门槛包括：

- 最小簇点数
- 最小簇宽度
- 最小簇高度
- 最小整体 extent
- 最大细长度 `aspect_ratio`
- 最小持续帧数
- 最小位移量
- 最小速度门槛


### 6.4 Baseline

当前还保留一条 baseline 对照链路：

- 帧差分
- 聚类
- 跟踪

它仍在后台运行，但前端已隐藏，不作为主显示。


## 7. 多目标支持

### 7.1 LD2450

LD2450 当前支持最多 3 个目标输出。

### 7.2 LD19P

LD19P 的 PC 算法当前支持多目标：

- 一个扫描中可形成多个候选簇
- 多个簇可进入多目标跟踪
- 最终可输出多个 `target_id`

限制：

- 靠太近的两个目标仍可能并成一个簇
- 很小、很慢或持续时间短的目标可能被滤掉


## 8. 运行方法

### 8.1 启动服务器

```powershell
cd C:\Users\thorn\Downloads\Tentacleslab\PlatformIO\Projects\ld2450_esp32
python -m uvicorn server.app:app --host 0.0.0.0 --port 8000 --reload
```

访问：

- 本机：`http://127.0.0.1:8000/`
- 局域网：`http://<PC_IP>:8000/`


### 8.2 烧录 ESP32

```powershell
cd C:\Users\thorn\Downloads\Tentacleslab\PlatformIO\Projects\ld2450_esp32
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -t upload --upload-port COM4
```


### 8.3 启动 LD19P Reader

先关闭 `LdsPointCloudViewer`，避免占用串口。

```powershell
cd C:\Users\thorn\Downloads\Tentacleslab\PlatformIO\Projects\ld2450_esp32
python server\ld19_reader.py COM4 --server-url http://127.0.0.1:8000 --node-id ld19_backyard
```


### 8.4 远程 PC2 节点

如果 `LiDAR` 接在第二台电脑 `PC2` 上，可以让它通过 Wi‑Fi 上传到主服务器 `PC1`：

```powershell
cd C:\Users\thorn\Downloads\Tentacleslab\PlatformIO\Projects\ld2450_esp32
python server\ld19_reader.py COM4 --server-url http://192.168.1.128:8000 --node-id ld19_backyard_pc2
```

上传的逻辑节点：

- 原始点云：`ld19_backyard_pc2`
- 主算法目标：`ld19_backyard_pc2__motion`
- baseline：`ld19_backyard_pc2__baseline`

网页当前隐藏 baseline。


## 9. 调试建议

### 9.1 LD2450

串口调试命令：

- `h`：帮助
- `r`：原始帧 hex
- `p`：PLOT 输出
- `v`：dashboard

低延迟模式建议：

- `PLOT` 关闭
- `dashboard` 关闭


### 9.2 LD19P

优先观察：

- 原始点云是否连续稳定
- `M` 目标是否连续存在
- 墙体/树木边缘是否误触发

当前最有效的优化方向：

- 收紧目标成立门槛
- 再考虑 ROI


## 10. 待办

- 为 `M` 算法增加参数可视化调节
- 为 `Floor C` 增加 `Raw / Motion / Both` 开关
- 继续压制墙体/树木边缘误触发
- 增加 LiDAR ROI / mask
- 评估是否需要把 `LD19P` 迁移到 `ESP32`
- 最后阶段再做：
  - `ESP32 AP setup mode + Preferences`
  - 设备首次配网页面
  - Wi‑Fi / Server URL 持久化

# ref
https://wiki.youyeetoo.com/en/Lidar/D300