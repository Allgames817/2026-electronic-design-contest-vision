# 2026 Electronic Design Contest Vision

本仓库面向 2026 年电赛 H 题，代码主要分为两个独立功能：

1. **无线图传**：以 `mjpeg_steel_ball_server.py` 为核心，将 K230 摄像头画面通过 Wi-Fi 传到浏览器。
2. **钢球识别**：以 `steel_ball_best.py` 为核心，识别钢球、计算位置并通过串口输出结果。

两个程序都会使用摄像头和显示资源，通常根据当前任务选择其中一个作为运行入口。

## 功能一：无线图传

### 核心文件

| 文件 | 用途 |
| --- | --- |
| `mjpeg_steel_ball_server.py` | 连接 Wi-Fi，提供 MJPEG 实时画面、检测结果叠加和 H.264 测试录像 |
| `main.py` | 上电后自动启动 `mjpeg_steel_ball_server.py`，异常退出时自动重试 |
| `mjpeg_http_server.py` | 不带钢球识别功能的基础 MJPEG 图传测试程序 |

### 使用方法

1. 打开 `mjpeg_steel_ball_server.py`，填写本地 Wi-Fi 信息：

   ```python
   WIFI_SSID = "YOUR_WIFI_SSID"
   WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
   ```

2. 准备与脚本匹配的单类 YOLO 模型，并在 K230 上保存为：

   ```text
   /sdcard/steel_ball.kmodel
   ```

3. 将 `main.py` 和 `mjpeg_steel_ball_server.py` 放到 K230 对应的启动目录。
4. 启动设备，在串口日志中查看 K230 获得的 IP 地址。
5. 手机或电脑连接同一局域网，在浏览器中访问：

   ```text
   http://设备IP:8080/
   ```

6. 录像控制页面为：

   ```text
   http://设备IP:8080/control
   ```

录像文件保存在 K230 的 `/data/Video` 目录。

## 功能二：钢球识别

### 核心文件

| 文件 | 用途 |
| --- | --- |
| `steel_ball_best.py` | 使用 YOLOv8 模型检测钢球，计算钢球中心与有符号位置，并通过 UART1 发送结果 |
| `best.kmodel` | `steel_ball_best.py` 使用的单类钢球检测模型 |
| `center.cfg` | 保存机械中心标定结果 |
| `PLUS.cfg` | 保存正方向参考位置标定结果 |
| `Minus.cfg` | 保存负方向参考位置标定结果 |
| `k230_uart_coordinate_sender.py` | UART2 接线和固定坐标发送测试程序 |

### 使用方法

将以下文件复制到 K230 SD 卡根目录：

- `steel_ball_best.py`
- `best.kmodel`
- `center.cfg`
- `PLUS.cfg`
- `Minus.cfg`

在 CanMV IDE 中运行 `steel_ball_best.py`。程序会完成钢球检测，在屏幕上显示结果，并通过 UART1 输出钢球中心坐标、置信度和位置。

默认串口参数为 115200 波特率、8N1。检测成功时输出：

```text
BALL,cx,cy,confidence
POS,signed_position_cm
```

未检测到钢球时输出：

```text
BALL,-1,-1,0
POS,NA
```

首次使用或机械结构发生变化后，应重新完成中心点、正方向和负方向的位置标定。

## 数据采集辅助程序

| 文件 | 用途 |
| --- | --- |
| `capture_ball_dataset.py` | 采集钢球图像及半管位置标定图像 |
| `capture_yolo_3class_final.py` | 采集 ball、zero、limit 三类 YOLO 原始数据，并生成 `manifest.csv` |

采集模式、时间间隔、图片数量、翻转方向和保存路径可以在脚本顶部配置。

## 运行环境

- CanMV K230 开发板
- CanMV K230 MicroPython 固件 v1.8
- OV5647 摄像头
- 800 × 480 ST7701 LCD
- 可选：TI MSPM0 控制器，用于接收 UART 坐标和位置数据

## 注意事项

- K230 端脚本依赖 CanMV 固件内置模块，不能直接在普通 CPython 环境运行。
- 模型输入尺寸、类别顺序和脚本配置必须与训练及模型转换时保持一致。
- K230 与外部控制器连接时必须共地，并确认 UART 引脚映射和电平兼容。
- 仓库代码中的 Wi-Fi 配置使用占位符，不要将真实 Wi-Fi 密码提交到公开仓库。
