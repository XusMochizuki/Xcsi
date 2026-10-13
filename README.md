# esp-csi 工作区

基于官方 [espressif/esp-csi](https://github.com/espressif/esp-csi) `examples/get-started` 的 ESP32-C6 CSI 实验。

## 目录

| 路径 | 说明 |
|---|---|
| `upstream/` | 官方仓库浅克隆（只读参考，**不在本仓库跟踪**，需要时自行 `git clone --depth 1 https://github.com/espressif/esp-csi.git upstream`）。基于 commit `8633d67`（2026-04-22） |
| `firmware/csi_send/` | 发射端固件，派生自 esp-csi `examples/get-started/csi_send` |
| `firmware/csi_recv/` | 接收端固件，派生自 esp-csi `examples/get-started/csi_recv`（二进制帧输出） |
| `tools/` | `csi_web.py`+`csi_web.html` 网页采集、`csi_unpack.py` bin→npy、`csi_plot.py` 分析图、`csi_frame.py` 帧解析、`loss_test.py` 三路采集/丢包、`phase_analysis.py` 芯片间相位差、`boot_scan.py` 多次重启扫描、`angle_fit.py` 角度拟合、`flash_rx.sh` 批量烧录、`usb_attach.sh` 自动 attach、`devices.sh` 设备表 |
| `runs/` | 采集数据（不在本仓库跟踪） |
| `docs/hardware.md` | 硬件与端口映射 |
| `docs/issues.md` | 已知问题与待测项 |
| `docs/phase.md` | 三芯片相位差实测记录、B 路硬件故障排查、PLL 重锁 / 整数分频实验（第三轮） |
| `research/` | ESP-SDR（ESPARGOS 原始 IQ）评估：IQ→CSI 处理链、合成测试、PLL 实验脚本，结论见 `research/README_RESEARCH.md`；`research/repro/` 为可复现测试 t0–t4（自动发现设备、自动判定、生成报告） |

## 许可

`firmware/` 派生自 [espressif/esp-csi](https://github.com/espressif/esp-csi)，遵循 Apache-2.0（见 `firmware/LICENSE`，源文件保留原 SPDX 头）。
其余部分（`tools/`、`docs/`、`research/`）的许可证尚未确定。`research/ext/`（ESP-SDR，GPL-3.0）只在本地克隆，不纳入本仓库。

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
- RF PLL 命令（csi_recv 和 csi_send 都有，调用私有 libphy 函数 `phy_set_freq`）：`F` + 4 位数字（MHz，2300–2500）改频，
  `r` 在当前频点重锁。RX 在下一帧 CSI 回调里执行，所以如果 TX 已经不在同一频点、收不到帧，命令就不会生效（先调 RX 再调 TX）。
  帧头 `lo_mhz`（原 reserved1，0 = 没改过），`flags` 第 8–15 位为重锁计数。
- csi_send：`P` + 4 位数字设置发包速率（1–1000 Hz，1 ms tick）。

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

### 网页采集 → npy → 分析图

```sh
uv run --python 3.12 --with pyserial,numpy tools/csi_web.py            # 打开 http://localhost:8000
uv run --python 3.12 --with numpy tools/csi_unpack.py runs/<name>      # → runs/<name>/npy/
uv run --python 3.12 --with numpy,matplotlib tools/csi_plot.py runs/<name>/npy --sc 20
```

- `csi_web.py`：每 2 s 扫描 `/dev/serial/by-id` 的 Espressif 口，发出 CSI 帧的就是接收端，`98:88:E0:71:A6:44` 是 TX；
  重新枚举后自动重连，不复位芯片。**已被其它进程打开的口会跳过**（页面显示占用进程），自己打开的口设 `TIOCEXCL`，
  之后启动的脚本会直接报 busy，而不是两个进程悄悄分抢同一个字节流。所以跑 `loss_test.py` 等脚本前先停掉网页服务。
- 页面：选一个子载波（输入 / 滑条 / 点击幅值谱），每个接收端一条幅值曲线，按 `tx_seq` 对齐，断线即丢包；
  卡片显示帧率、近 10 s 丢包、RSSI、AGC/FFT、温度、LO。服务端每 50 ms 只推选中子载波的幅值，幅值谱 4 Hz。
  按钮：`F`+MHz 改频（先发 RX、0.3 s 后发 TX）、增益锁定 `g` / 自动 `a`。
- 记录：每个 CRC 正确的帧原样写入 `runs/<时间>-<名字>/<MAC>.bin`（帧头 + CSI + CRC，与 `loss_test.py` 相同，
  `csi_frame.load`、`phase_analysis.py` 都能直接读），加 `meta.json`（设备、标签、帧数、tx_seq 范围、LO 请求）。
- `csi_unpack.py`：按 TX 的 `tx_seq` 把各路对齐到同一网格（默认取各路都覆盖的区间，`--span union` 取并集）。
  丢失的帧默认线性插值（幅值线性；相位沿两侧帧之间的短弧；RSSI/AGC 等线性；两端取最近值），`--interp none` 填 NaN。
  输出 `csi.npy` complex64 `[设备, 帧, 256]`（bin k = 子载波 k−128）、`valid.npy`（真实收到 = True）、`tx_seq.npy`、
  `ts_us.npy`、`rssi/agc/fft_gain/temp_c/noise_floor.npy`、`subcarrier.npy`、`tone_mask.npy`（|k| = 2…122）、`info.json`。
  每包的绝对相位带随机公共项（CFO/定时），插出来的绝对相位只是占位；幅值和芯片间相位差才有意义，需要时用 `valid` 剔除。
- `csi_plot.py`：选中子载波幅值（空心点 = 插值帧）、丢包栅格、各路 |H| 时间×子载波热图、平均幅值谱 ±1σ、RSSI、AGC、
  两两相位差（只用两路都收到的帧，纵轴以圆均值为中心）。
