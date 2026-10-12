# esp-csi 工作区

基于官方 [espressif/esp-csi](https://github.com/espressif/esp-csi) `examples/get-started` 的 ESP32-C6 CSI 实验。

## 目录

| 路径 | 说明 |
|---|---|
| `upstream/` | 官方仓库浅克隆（只读参考，**不在本仓库跟踪**，需要时自行 `git clone --depth 1 https://github.com/espressif/esp-csi.git upstream`）。基于 commit `8633d67`（2026-04-22） |
| `firmware/csi_send/` | 发射端固件，派生自 esp-csi `examples/get-started/csi_send` |
| `firmware/csi_recv/` | 接收端固件，派生自 esp-csi `examples/get-started/csi_recv`（二进制帧输出） |
| `tools/` | `csi_frame.py` 帧解析、`loss_test.py` 三路采集/丢包、`phase_analysis.py` 芯片间相位差、`boot_scan.py` 多次重启扫描、`angle_fit.py` 角度拟合、`flash_rx.sh` 批量烧录、`usb_attach.sh` 自动 attach、`devices.sh` 设备表 |
| `runs/` | 采集数据（不在本仓库跟踪） |
| `docs/hardware.md` | 硬件与端口映射 |
| `docs/issues.md` | 已知问题与待测项 |
| `docs/phase.md` | 三芯片相位差实测记录、B 路硬件故障排查 |

## 许可

`firmware/` 派生自 [espressif/esp-csi](https://github.com/espressif/esp-csi)，遵循 Apache-2.0（见 `firmware/LICENSE`，源文件保留原 SPDX 头）。
其余部分（`tools/`、`docs/`）的许可证尚未确定。

## 相对 upstream 的改动

- `WIFI_BW_HT20/HT40` → `WIFI_BW20/BW40`（ESP-IDF v6 删除了旧枚举名）。
- 射频配置（TX/RX 一致）：802.11ax HE20（协议 11b|g|n|ax，ESP-NOW phymode `WIFI_PHY_MODE_HE20`，MCS0），
  信道 1，国家码 `US`，发包 100 Hz（`xTaskDelayUntil` 固定周期；原版 `usleep` 实际只有约 91 Hz）。
- 控制台：USB-Serial-JTAG（`sdkconfig.defaults`），不再走 UART0。
- csi_recv 输出改为二进制帧（不再 `ets_printf` 字符串），格式见 `firmware/csi_recv/main/csi_frame.h`：
  48 字节头 + 原始 CSI（HE20 为 512 字节，`[imag0, real0, imag1, real1, ...]` int8，未做增益补偿）+ CRC-32。
  CSI 回调只拷贝入队（深度 16），由独立任务经 USB-Serial-JTAG 驱动发送；队列满即丢弃，主机端表现为 `frame_seq` 跳号。
  只采集 HE SU（`acquire_csi_su`），其它类型关闭。
- csi_recv：每次启动都做 PHY 全校准（`CONFIG_ESP_PHY_CALIBRATION_AND_DATA_STORAGE=n`，不读取 NVS 里存储的校准数据）。
- csi_recv：主机单字节命令 `g`（采 100 帧取中位数后锁定 RX 增益，帧头 flags bit1 置位）/ `a`（恢复自动增益）。
- 帧头 `temp_c`（原 reserved0）：芯片内部温度，℃，1 Hz 更新。

## 环境

- ESP-IDF v6.0.2：`~/.espressif/v6.0.2/esp-idf`（EIM 安装）
- 激活（fish）：`source ~/.espressif/tools/activate_idf_v6.0.2.fish`
  bash：`source ~/.espressif/tools/activate_idf_v6.0.2.sh`

## 编译 / 烧录

```sh
cd firmware/csi_send          # 或 csi_recv
idf.py set-target esp32c6     # 首次
idf.py build
idf.py -p /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00 flash
```

WSL 里用串口：每个设备先在 Windows **管理员** shell 里 `usbipd bind --busid <id>` 一次，
然后启动自动重连（烧录/复位后 USB 会重新枚举，需要重新 attach）：

```sh
tools/usb_attach.sh                 # 默认 1-2（TX, COM9）
tools/usb_attach.sh 2-1 2-2 2-3     # RX
tools/usb_attach.sh stop
```

`/dev/ttyACMx` 编号随 attach 顺序变化，用固定路径 `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_<MAC>-if00`。

## 主机脚本

二进制帧接收/统计/保存（`tools/csi_frame.py`）：

```sh
uv run --python 3.12 --with pyserial tools/csi_frame.py /dev/serial/by-id/...-if00 [--out rx.bin]
uv run --python 3.12 tools/csi_frame.py --file rx.bin
```

每秒输出：`frames`、`local_drop`（`frame_seq` 跳号：设备队列满，或被 CRC 拒收，对照 `crc_err`）、
`air_loss`（`tx_seq` 跳号减去 local_drop：空中丢失）、`crc_err`、平均 RSSI、CSI 长度、bb_format（4 = HE SU）。
Python 中 `from csi_frame import load` 返回头列表和 complex64 `[帧, 子载波]` 数组。

三路丢包测试（先 `tools/flash_rx.sh` 烧录全部接收端）：

```sh
uv run --python 3.12 --with pyserial tools/loss_test.py --seconds 120 --out runs/<name>
```

输出每秒各路帧数，结束后给出每路 recv / loss% / air / local / crc / 最长连续丢包 / RSSI，
以及三路同时收到的比例；`--out` 保存原始流和 `summary.json`。

相位分析（详见 `docs/phase.md`）：

```sh
uv run --python 3.12 --with numpy,matplotlib tools/phase_analysis.py runs/<name>       # 输出 phase.png、phase.json
uv run --python 3.12 --with numpy,matplotlib,pyserial tools/boot_scan.py --boots 20   # 重启扫描
```

`loss_test.py` 打开串口默认**不复位**芯片；`--reset` 主动复位，`--lock-gain` 锁定增益。
