# 硬件

## 发射端（TX）

- 1 × ESP32-C6，原生 USB-Serial-JTAG（303a:1001）
- Windows：COM9，usbipd busid `1-2` → WSL `/dev/ttyACM0`
- 芯片 MAC `98:88:E0:71:A6:44`（USB 序列号）→ `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_98:88:E0:71:A6:44-if00`
- Wi-Fi 发送 MAC 被固件改成 `1a:00:00:00:00:00`（接收端按它过滤）
- 固件：`firmware/csi_send`

## 接收端（RX）

- 定制板：3 × ESP32-C6，**共用同一颗晶振**
- 外置 FPC 天线，天线间距 **61 mm**
  （2.4 GHz λ≈122 mm，61 mm ≈ λ/2）
- 设计初衷：三路同时接收同一帧，利用通道间相位差辅助 HAR / 室内定位（AoA 类思路）。
  后续怀疑效果有限，需要更多测试再定。
- 三颗芯片接到同一个 USB hub，各走原生 USB-Serial-JTAG
- 固件：`firmware/csi_recv`

| 板上天线座 | Windows | usbipd busid | MAC（USB 序列号） |
|---|---|---|---|
| A | COM17 | 2-2 | 98:88:E0:76:5D:68 |
| B | COM16 | 2-1 | 98:88:E0:76:07:C4（**B 位置 PCB 射频通路弱**，见 docs/phase.md 第 0–0d 节） |
| C | COM15 | 2-3 | 98:88:E0:76:4D:04 |

2026-10-12 B、C 两颗芯片互换过（4D:04 原在 B，07:C4 原在 C）。Windows COM 号跟着芯片（USB 序列号）走，busid 跟着板上位置走。

busid/COM 与 MAC 的对应来自 Windows PnP 位置信息（Hub #2 端口 N = busid 2-N），2026-10-12。
MAC 是固定身份，busid/COM 重新插拔后可能变化。设备表在 `tools/devices.sh`、`tools/loss_test.py`。
天线座与芯片的对应：2026-10-12 逐个拔天线确定（拔掉后只剩座子 RSSI 约 −75 dBm）。
物理排列（2026-10-12 用户确认）：天线贴在打印的支架上，PCB 装在背面。
从 PCB 一侧看：**B（左）– A（中）– C（右）**；从天线一侧看左右相反。相邻间距 61 mm。
因此 B–C（COM15–COM16）是两端那一对（间距 122 mm，角度测试中斜率应为相邻对的 2 倍），
A–B（COM17–COM15）和 A–C（COM17–COM16）是相邻对。
