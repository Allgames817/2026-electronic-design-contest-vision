# 2026 Electronic Design Contest Vision

基于 CanMV K230 的视觉实验与钢球检测项目，包含数据采集、YOLO 模型推理、位置标定、串口坐标发送、MJPEG 预览和 H.264 测试录像等功能。

## 运行环境

- CanMV K230 开发板
- CanMV K230 MicroPython 固件 v1.8
- OV5647 摄像头
- 800 × 480 ST7701 LCD
- 可选：TI MSPM0 控制器，用于接收 UART 坐标或位置数据

## 主要文件

| 文件 | 用途 |
| --- | --- |
| `capture_ball_dataset.py` | 采集钢球数据和半管位置标定图像 |
| `capture_yolo_3class_final.py` | 采集 ball、zero、limit 三类 YOLO 数据集 |
| `steel_ball_best.py` | 使用 `best.kmodel` 检测钢球、完成位置标定并通过 UART1 发送结果 |
| `k230_uart_coordinate_sender.py` | UART2 固定坐标发送与接线测试 |
| `mjpeg_http_server.py` | 基础摄像头 MJPEG 网页预览 |
| `mjpeg_steel_ball_server.py` | 钢球检测、MJPEG 预览和 H.264 测试录像 |
| `main.py` | 上电自动启动 `mjpeg_steel_ball_server.py` |
| `center.cfg`、`PLUS.cfg`、`Minus.cfg` | 机械中心及正负位置标定参数 |
| `examples/` | CanMV K230 示例程序、模型和测试资源 |
| `Video/` | 实验图片和视频 |

## 快速开始

### 1. 单模型钢球检测与串口输出

将以下文件复制到 K230 SD 卡根目录：

- `steel_ball_best.py`
- `best.kmodel`
- `center.cfg`
- `PLUS.cfg`
- `Minus.cfg`

在 CanMV IDE 中运行 `steel_ball_best.py`。默认串口参数为 115200 波特率、8N1，程序会输出：

```text
BALL,cx,cy,confidence
POS,signed_position_cm
```

未检测到钢球时输出：

```text
BALL,-1,-1,0
POS,NA
```

### 2. 网页预览与录像

1. 打开 `mjpeg_steel_ball_server.py`，填写本地 Wi-Fi 名称和密码。
2. 准备与脚本匹配的单类 YOLO 模型，并在 K230 上保存为 `/sdcard/steel_ball.kmodel`。
3. 将 `main.py` 和 `mjpeg_steel_ball_server.py` 复制到设备对应的启动目录。
4. 启动后在串口日志中查看设备 IP，并在同一局域网浏览器中访问 `http://设备IP:8080/`。
5. 录像控制页面位于 `http://设备IP:8080/control`，录像保存到 `/data/Video`。

仓库中的 Wi-Fi 配置使用占位符，提交代码前不要写入真实密码。

### 3. 数据采集

- `capture_ball_dataset.py`：通过 `CAPTURE_SET` 选择普通钢球数据或半管位置标定数据。
- `capture_yolo_3class_final.py`：采集与推理画面一致的 640 × 360 三分类原始图片，并生成 `manifest.csv`。

采集模式、间隔、图像数量、翻转方向和保存路径均可在脚本顶部配置。

## 未纳入仓库的内容

以下内容保留在本地，不上传到 GitHub：

- 固件镜像和安装程序
- `tmp/` 中的依赖缓存
- 本地 Wi-Fi 凭据与环境文件
- 板端本地文件 `steel_ball.py` 和 `steel_ball.kmodel`

## 注意事项

- K230 端脚本依赖 CanMV 固件内置模块，不能直接在普通 CPython 环境运行。
- 模型输入尺寸、标签顺序和脚本配置必须与训练、转换时保持一致。
- K230 与外部控制器连接时必须共地，并确认 UART 引脚映射和电平兼容。
- 首次运行位置检测前，请检查或重新生成中心点及正负位置标定文件。
