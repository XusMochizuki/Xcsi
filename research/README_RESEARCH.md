# ESP-SDR 对 ESP32-C6 三芯片 CSI（尤其是相位同步）有没有帮助（2026-10-12）

证据级别：**VERIFIED** = 本台架硬件实测；**SIMULATED** = 合成信号；**CODE** = 从代码确认；
**HYPOTHESIS** = 推测；**BLOCKED** = 无法验证。

## 结论（先看这里）

1. **IQ 捕获本身不能解决芯片间相位同步。** IQ 和原生 CSI 经过同一个 RF PLL / LO；
   每次启动的随机偏置就是 PLL 锁定相位（VERIFIED：同一次启动内只重锁 RX PLL 就能复现，只重锁 TX 不会改变）。
   换成 IQ 采样，这个偏置一样存在。
2. **ESP-SDR 真正有用的是它揭示的私有 PHY 接口。** 用 `phy_set_freq()` 把三颗 RX 和 TX 调到
   **整数分频频点 2440 MHz（61 × 40 MHz）**后，芯片间相位偏置从"每次锁定都连续随机"变成
   **θ0 + m·90°**，θ0 跨重启可重复（同一时段 ±10°）。剩下的只是 4 选 1，不再需要连续标定（VERIFIED，见 `docs/phase.md` 第三轮）。
   这是本轮最有价值的结果，而且只改固件。
3. **从 C6 原始 IQ 能正确重建 L-LTF 和 2x HE-LTF 的 CSI**（VERIFIED + SIMULATED）。
   重建的 HE-LTF CSI 与原生 CSI 质量**相当**（残差 −20.0 vs −20.4 dB，相位 σ 3.4° vs 3.7°，平均信道形状相关 0.999）。
   L-LTF（两个符号平均，单音调功率更高）更干净：−25.2 dB、2.3°。没有看到 IQ 路线带来质量上的跨越。
4. **IQ 路线的产出率太低，没法做连续 CSI**：C6 只支持软件触发、80 MS/s、单次最多 16380 点（205 µs），
   每秒约 16 次（空口占空比 0.32%）。TX 发到 1 kHz 时也只有约 1.6 个可用包/秒；TX 100 Hz 时约 0.2 个/秒。
   300 Hz 连续 CSI 只能靠原生 CSI。
5. 三颗芯片同时抓同一个包的 IQ，在 esp-sdr 里**做不到**：没有包触发，也没有芯片间共享触发。
   ESPARGOS 阵列用的是"参考包时间戳锚点 + 定时网格"同步，但实现在它没有开源的传感器固件里（CODE：只有
   `pyespargos/espargos/iq_sync.py` 主机侧代码可见）。

## 1. ESP-SDR 在 C6 上怎么捕获 IQ（CODE，已实测 VERIFIED）

| 项 | 内容 | 依据 |
|---|---|---|
| 机制 | 调试 dump 通路绕过 Wi-Fi modem：`0x600a9004` 控制（低位 = 点数，bit31 使能，bit19 软件触发脉冲，bit18 完成），`0x600a9008` 选源（source 15），`SRAM_OWNER_REG 0x60095004` 把 SRAM bank 切给 dump | `main/targets/esp32c6/chip.h: stock_capture()` |
| 缓冲 | `0x40840000`，预留 `0x40820000–0x40860000`（256 KB，C6 的 HP SRAM 一半） | `chip.h`、`sram_guard.ld` |
| 采样 | 只有 80 MS/s（rate 0）；每点 20 bit（I = bit0–9，Q = bit10–19，有符号 10 bit）；256–16380 点 | `receiver.c` 命令解析、`IQ_WORDS 16380` |
| 触发 | 只有软件触发，每次拍一张快照；没有 Wi-Fi 包触发；USB 传输期间不采样 | `receiver.c: acquire_iq()`、`RXRUN` |
| 依赖 | `CONFIG_ESP_PHY_ENABLE_CERT_TEST`（librftest）、`chip_v7_set_chan`、`phy_set_freq`、`rom_chip_i2c_*`、`phy_param` 偏移（和 PHY 库版本绑定） | `sdkconfig.defaults`、`rx_recalibration.c` |
| IDF | 锁定 esp-idf `25fe69f9`（2026-07-11 master），PHY 库 `e294ff0` ≠ v6.0.2 的 `59c1234`，所以必须用锁定版本编译 | `firmware-targets.json` |
| 和 Wi-Fi 的关系 | esp-sdr 运行时 Wi-Fi 是 NULL 模式 + 混杂模式，PHY 被切到 rftest 工作方式；它是一个独立固件，不能和 csi_recv 同时运行 | `receiver.c: app_main()` |
| Q 极性 | **dump 口的 Q 极性是反的**：不取共轭的话频谱镜像、CFO 变号、CIR 被打散（实测）；pyespargos 对 C61 有同样的说明 | `pyespargos/espargos/iq_packet.py:104`；本次实测 |
| 直流 | 实测 Q 路直流约 −75 码（满量程 ±512），需要先去直流 | VERIFIED |
| AGC | 硬件 AGC 下，包头先削波，约 3 µs 后增益下调（48 个突发里 38 个削波）；做包分析用手动增益（本台架 `GAIN MANUAL 20`） | VERIFIED |

实测吞吐：`CAP20 16380 0` 每次 208 µs，主机端 15–16 次/秒（数据 40 KB/次，USB-Serial-JTAG）。

## 2. IQ → CSI 处理链（`research/dsp/wifi_ofdm.py`）

80 → 20 MS/s（remez 抽取滤波器，通带平坦到 9.7 MHz：2x HE-LTF 的边缘音调在 ±9.53 MHz，
默认的 `resample_poly` 会把它们衰减约 35%）→ L-STF 自相关检测（加噪底门限，防止直流/单音误检）+ 粗 CFO
→ L-LTF 互相关精定时 + 精 CFO → L-LTF CSI（52 个音调，两符号平均，用两符号之差估噪声）
→ L-SIG 等化后算 EVM → 2x HE-LTF CSI（122 个偶数音调 ±2…±122，GI 1.6 µs）。

HE-LTF 的 ±1 序列表手头没有经过核实的版本，所以**没有写死**。每个音调的符号由 L-LTF 信道插值来判决，
并用"每包判出的符号图样必须完全相同"来检验。实测 63/63 个包 122 个音调全部一致。
包格式来自原生 CSI 的 HE-SIG-A：HE SU、MCS0、GI+LTF = 2（2x LTF + 1.6 µs GI）。
另外，原生 HE CSI 的 256 个 bin 里只有偶数音调是实测的，其余是插值。

合成测试（SIMULATED，`research/tests/test_wifi_ofdm.py`，11 项通过）：3 径信道、分数时延、CFO ±95 kHz、
10 bit 量化、直流 −75j、AWGN；校验检测、定时 ±2 采样、CFO < 1.5 kHz、CSI NMSE 跟随 1/SNR、
HE 符号恢复、纯噪声不误检、截断包拒收。

真实数据（VERIFIED，COM17/5D:68，2412 MHz，手动增益 20，TX 1 kHz）：600 次捕获检出 87 个包，
其中 63 个属于我们的 TX（L-SIG 比特一致）；CFO −27.4 ± 1.6 kHz；L-LTF SNR 中位 26.7 dB。
**L-SIG EVM 约 18.5%（−14.7 dB）**，在各子载波上平坦，与定时无关，也不是 IQ 镜像。
HYPOTHESIS：TX 在 MCS0 下以最大功率发射，PA 非线性；这部分失真在两个 L-LTF 符号里相同，SNR 估计看不到。待降低 TX 功率验证。

## 3. 质量对比（`research/benchmarks/csi_quality.py`，VERIFIED，静态场景，前后分开采集）

| 数据 | 包数 | 残差（越低越好） | 相位 σ 中位 | 幅度 CV |
|---|---|---|---|---|
| IQ L-LTF（52 音调） | 63 | −25.2 dB | 2.33° | 0.039 |
| IQ 2x HE-LTF（122 音调） | 63 | −20.0 dB | 3.42° | 0.066 |
| 原生 HE CSI，实测偶数音调（122） | 3212 | −20.4 dB | 3.71° | 0.069 |
| 原生 HE CSI，全部 bin（242，含插值） | 3212 | −21.6 dB | 3.10° | 0.061 |

两组数据不可能来自同一个包（一次只能跑一个固件），是前后各采约 30–40 s。
平均信道形状相关：IQ HE-LTF 对原生 0.999，IQ L-LTF 对原生同频点 0.997，说明期间场景没有变。
原生是自动增益，IQ 是手动增益。结果数据：`research/reports/csi_quality_5D68.json`。

## 4. 回答任务书里的 10 个问题

1. **C6 怎样捕获原始 IQ、最大长度？** 调试 dump 通路，软件触发快照，80 MS/s，10 bit I/Q，最多 16380 点 = 204.75 µs（VERIFIED）。
2. **能否从 IQ 重建 L-LTF / HE-LTF CSI？** 能。L-LTF 和 2x HE-LTF 都能，并且和原生 CSI 一致（VERIFIED）。
3. **比原生 CSI 更好吗？** HE-LTF 上持平；L-LTF 好约 5 dB，但那是另一个训练字段（当前 csi_recv 配置只取 HE SU 的 CSI；C6 原生能否对同一个 HE 包同时给出 L-LTF 没测）。
   代价是产出率差 2–3 个数量级。没有实质提升（VERIFIED）。
4. **能否消除三颗 C6 的随机初相？** 用 IQ 不能。用整数分频频点（2440 MHz）可以把它降成 90° 的 4 选 1，
   θ0 在同一时段跨重启可重复（VERIFIED）；长期是否稳定还没在固定几何下验证。
5. **IQ 和正常 CSI 能否交替/同时运行？** esp-sdr 是独立固件，不能同时运行。若要在 csi_recv 里嵌入快照捕获，
   需要切 PHY 工作方式和 256 KB SRAM，原生 Wi-Fi 接收会中断；切回来是否引起 PLL 重锁（会重新随机化相位）未验证（HYPOTHESIS/BLOCKED）。
6. **能否保持 ≥300 Hz 连续 CSI？** IQ 路线不能（约 1.6 包/秒）。原生 CSI 可以（之前实测 100 Hz 发包时丢包 < 1%；300 Hz 本轮没测）。
7. **哪些 ESPARGOS 技术能直接移植？** 能直接用的：esp-sdr 的私有 PHY 调用和寄存器知识（`phy_set_freq`、PLL 块地址）、DC/AGC 处理经验。
   需要硬件支持的：ESPARGOS 的相干性来自**板上的参考信号注入**（参考发射 → 功分器 → 射频开关 → 每个传感器，见 `pyespargos/docs/combined-arrays.rst`），
   我们的板子没有这条通路。它的 IQ 定时网格同步在未开源的固件里，只能照原理重写。
8. **哪些要改硬件，哪些只改固件/软件？** 只改固件：整数分频频点 + 4 选 1 标定（已验证）、按 AGC 码的相位查表。
   要改硬件：板上参考信号注入（射频开关 + 功分器 + 参考源），这样每次启动都能自动、无几何依赖地完成标定；
   芯片间加一根 GPIO 触发线，才能三芯片同时抓 IQ。
9. **限制来自哪里？** 相位偏置：RF PLL（小数分频 SDM 初相 + LO 分频的 4 重模糊）。IQ 产出率：单次 SRAM 深度（205 µs）加 USB-Serial-JTAG 传输（每次约 60 ms）。
   IQ 和三芯片同步：没有包触发和共享触发（固件/硬件），不是射频本身的限制。
10. **只投入一项的话选什么？** **整数分频频点（2440 MHz）+ 每次启动一次粗标定（4 选 1）。** 只改固件，已经实测有效，
    能把"每次启动必须精确标定"变成"选一个离散状态，容差 ±45°"。下一步是在固定几何下验证 θ0 的长期稳定性和温度系数（见 NEXT STEPS）。

## 5. 下一步

1. 固定几何（TX 上支架，周围不动），在 2440 MHz 下隔几小时、改变温度（吹风）重复 `boot2440`，确认 θ0 是否长期可重复、温度系数多大。
2. 弄清 4 个状态的来源（LO ÷4 / 正交分频器的初相），找能读出或强制状态的寄存器（0x62 块已排除）。读得出来的话就完全不需要参考源。
3. 降低 TX 功率（`esp_wifi_set_max_tx_power`），复测 L-SIG EVM，验证 PA 非线性假设。
4. 测原生 CSI 在 300 Hz 下三路的丢包。
5. 如果做下一版硬件：加参考注入通路（ESPARGOS 方案），以及芯片间的 GPIO 触发线。

## 运行

**可迁移的测试脚本见 `research/repro/`（t0–t4，自动发现设备、自动判定、生成报告）。** 下面是本轮研究时用的原始命令。

```sh
# 固件（IDF v6.0.2）：csi_recv / csi_send 已含 F/r/P 命令
tools/flash_rx.sh
# PLL 实验
uv run --python 3.12 --with pyserial,numpy research/scripts/relock_test.py --freq 2440 --steps 10 [--reset] --out runs/NAME
uv run --python 3.12 --with numpy research/scripts/relock_analysis.py runs/NAME
uv run --python 3.12 --with numpy research/scripts/mod90_pool.py runs/A runs/B ...
uv run --python 3.12 --with numpy research/scripts/agc_phase.py runs/A ...
uv run --python 3.12 --with numpy research/scripts/run_summary.py runs/A ...

# ESP-SDR（GPL-3.0，克隆在 research/ext/，不纳入本仓库）
git clone https://github.com/ESPARGOS/esp-sdr research/ext/esp-sdr && git -C research/ext/esp-sdr submodule update --init
# 需要锁定的 IDF 25fe69f9（装在 ~/.espressif/esp-idf-25fe69f）
. ~/.espressif/esp-idf-25fe69f/export.sh
idf.py -C research/ext/esp-sdr -B research/ext/esp-sdr/build-c6 -DIDF_TARGET=esp32c6 \
  -DSDKCONFIG=research/ext/esp-sdr/build-c6/sdkconfig -DSDKCONFIG_DEFAULTS=$PWD/research/ext/esp-sdr/sdkconfig.defaults.esp32c6 build flash -p PORT
uv run --python 3.12 --with pyserial,numpy research/capture/sdr_capture.py PORT --count 600 --gain 20 --out runs/x.npz
uv run --python 3.12 --with numpy,scipy research/scripts/iq_to_csi.py --min-snr 15 runs/x.npz
uv run --python 3.12 --with numpy research/benchmarks/csi_quality.py --iq runs/x.csi.npz --native runs/<loss_test dir>

# 测试
uv run --python 3.12 --with numpy,scipy,pytest pytest -q research/tests
```

文件：`capture/sdr_capture.py`（esp-sdr 协议、IQ10 解包、CRC 校验）、`dsp/wifi_ofdm.py`、`dsp/synth.py`、
`tests/test_wifi_ofdm.py`、`benchmarks/csi_quality.py`、`scripts/`（PLL 实验和诊断工具）。
原始数据在 `runs/`（不纳入版本控制）：`*relock*`、`*boot2440*`、`20261012-sdr-*.npz`、`20261012-native-5D68-2412`。
