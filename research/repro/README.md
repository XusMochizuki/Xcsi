# 可复现测试脚本

ESP32-C6 多接收端 CSI 相位实验的独立测试脚本。每个脚本开头都有 PEP 723 依赖声明，`uv run <脚本>` 会自动装好依赖。
设备按 USB 序列号（即芯片 MAC）查找，不写死串口路径，Linux / WSL / Windows（COMx）都能用。
每个测试都会把原始数据存到 `runs/<时间>-<测试名>/`，并在同目录生成 `report.md` 和 `result.json`；
加 `--analyze <目录>` 可以离线重新分析，不需要硬件。

| 脚本 | 需要硬件 | 回答的问题 | 时长 |
|---|---|---|---|
| `t0_selftest.py` | 否 | IQ→CSI 处理链在合成信号上是否正确 | 10 s |
| `setup_bench.py` | 是 | 找设备、写 `bench.json`、检查固件是否支持 F/r 命令 | 15 s |
| `t1_relock.py` | 1 TX + ≥2 RX | 芯片间相位偏置是否由 RX 的 PLL 锁定决定（RX 重锁会变，TX 重锁不变） | 约 1.5 min |
| `t2_integer_n.py` | 同上 | 整数分频频点是否把偏置量化成格点（默认 2412 / 2440 / 2442 / 2420 MHz） | 约 4 min |
| `t3_boots.py` | 同上 | 格点原点 θ0 跨重启是否可重复 | 每次重启约 30 s |
| `t4_iq_csi.py` | 1 TX + 1 块刷了 ESP-SDR 的板 | 原始 IQ 能否重建 L-LTF / HE-LTF CSI，质量和原生 CSI 比如何 | 约 1.5 min |

## 准备

1. 固件：TX 刷 `firmware/csi_send`，RX 刷 `firmware/csi_recv`（ESP-IDF v6.0.2，见仓库 README）。
   两者都要带 `F`（改频）/ `r`（重锁）/ `P`（发包速率）命令，即 2026-10-12 之后的版本。
2. 晶振：频点的整数 / 小数判断按 40 MHz 晶振计算（`t2_integer_n.py` 里的 `CRYSTAL_MHZ`）。
3. 台架：TX 和所有天线固定不动，周围尽量没人走动。芯片间相位差本身包含传播信道，环境一动就会混进结果。
   WSL：先用 `usbipd attach --wsl --busid <id> --auto-attach` 把设备挂进来。

```sh
uv run research/repro/t0_selftest.py
uv run research/repro/setup_bench.py                                    # 列出设备
uv run research/repro/setup_bench.py --tx <TX 序列号> --names A B C      # 其余设备都当 RX；或用 --rx 指定
uv run research/repro/setup_bench.py --check                            # 必须输出 check passed
uv run research/repro/t1_relock.py
uv run research/repro/t2_integer_n.py
uv run research/repro/t3_boots.py --freq 2440 --boots 5
# t4：先把一块 RX 刷成 ESP-SDR（见 research/README_RESEARCH.md），再运行：
uv run research/repro/t4_iq_csi.py --sdr <该板序列号> --native runs/<t1 目录>/A.bin
```

`bench.json` 是本台架的配置（TX 98:88:E0:71:A6:44，RX A/B/C）。换台架时用 `setup_bench.py` 重新生成。

## 串口独占

同一个 tty 被两个进程同时读时，字节流会被分走，两边都会丢帧，而且不报错。
`common.open_port` 打开前会扫描 `/proc`，端口已被别的进程占用就报 busy 并列出占用进程；
打开后设置 flock 和 TIOCEXCL，之后再有进程打开会得到 EBUSY。
每份报告都有一节"data integrity"，按 `frame_seq` 跳号统计芯片到主机之间丢了多少帧；
任何一路超过 1% 就判定这次数据无效。

## 判据

- **t1**：RX 重锁各段均值的圆标准差 > 30°，且每一段 TX 重锁都在最后一段 RX 的 ±15° 以内，才判定为"PLL 决定"。
- **t2**：取 k ∈ {1, 2, 4, 8} 中满足 R_k ≥ 0.85 的最小值作为格点阶数，格点间隔 = 360°/k。R_k = |mean exp(j·k·θ_s)|，θ_s 为各次锁定的段均值。
  只调 RX 时链路必须中断，否则说明改频没有生效。
- **t3**：每次启动的 θ0 都在合并 θ0 的 ±360/(4k) 以内（k = 4 时为 ±22.5°），才判定为"可重复"。
- **t4**：至少 10 个属于我们 TX 的包（L-SIG 硬判与多数图样相差 ≤ 3 位），并且其中 ≥ 90% 的包在 ≥ 98% 的音调上与多数 HE-LTF 符号图样一致。
  这几条判据是在 2026-10-12 晚的数据上定下的：EVM 偏高时，原来"完全一致"的要求过严。

t2 / t3 同时给出原始结果和 **AGC 补偿**后的结果。AGC 补偿的做法：在同一次运行里，以各段均值为参考，
按最小二乘为每颗芯片的每个 AGC 码估一个相位偏移，见 `common.agc_phase_table`。
本台架 A 芯片的 AGC 在两组档位间切换时，相位相差约 20°。

## 本台架的参考结果

| 测试 | 2026-10-12 上午（环境安静） | 2026-10-12 深夜（重跑） |
|---|---|---|
| t1 | — | 三对都判定为 PLL 决定：RX 重锁圆标准差 73–151°，TX 重锁偏差 ≤ 9° |
| t2 2440 MHz | 90° 格点，R4 = 0.98–0.99 | A−B 0.97（补偿后），A−C 0.75，B−C 0.77：C 有约 ±10° 的额外抖动，有一次锁在格点外 +35° |
| t2 2420 MHz | 45° 格点，R8 = 0.6–0.88 | 没有格点 |
| t2 2412 / 2442 MHz | 随机 | 随机 |
| t3 2440 MHz | 8 次启动，偏差 ≤ ±11° | 5 次启动（00:46 重跑），全部可重复，原始值最大偏差 6.8 / 4.2 / 3.2°。之前 23:54 那次的 boot00 被另一个进程同时读串口，丢了 15–27% 的帧，作废 |
| t4 | 63 包，HE 图样 100% 一致，IQ HE-LTF −20.0 dB vs 原生 −20.4 dB | 增益 20：23 包，判定有效；原生同时段也变差（−14.4 dB） |

上午的数据来自 `research/scripts/` 下的早期脚本（`runs/*relock*`、`runs/*boot2440*`），方法相同。
深夜的环境明显更嘈杂，原生 CSI 也一起变差。所以 2440 MHz 格点结论的稳健性还需要在安静、固定的条件下多跑几次 t2 / t3。
