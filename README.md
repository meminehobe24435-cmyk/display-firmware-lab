# display-firmware-lab

> 显示器固件（monitor / scaler MCU）每天要处理的五件事，各实现一遍并量化。
> **纯 C99 + libc，零第三方依赖**，`make test` 一条命令跑完 **340 项**自检断言。

<p align="center">
  <b>EDID 解析 · DDC/CI 通信 · 输入源切换 · OSD 菜单 · 背光 PWM</b>
</p>

---

## 先说边界（重要，请先读这一段）

**这是一个纯逻辑 / 数值仿真项目。**

1. **没有真实显示器、没有 EDID EEPROM、没有 I2C/DDC 总线、没有 scaler 芯片、没有背光驱动电路。**
   全部数据由本仓库的代码计算得到。
2. 协议实现依据**公开规范**（VESA E-EDID 1.4、DDC/CI / MCCS 的常见子集），
   **不是完整实现**：不实现 **CEA-861 扩展块**、不做 **HDCP / Scramble**、不做 **Type-C 的 PD 协商**、
   不做 DisplayPort 链路训练。
3. 参数是**典型设定值**（例如 12-bit PWM、20 kHz、gamma 2.2、冷平台 Rth/Cth、VCSEL 的 T0 等），
   **不是任何厂商机型的实测数据**。
4. **不是产品代码**：在 PC 上编译运行，**没有跑在真实 MCU / scaler 上**，
   没有实时性、中断、I2C 时序与 DMA 验证。
5. **没做的**：真实的显示器兼容性测试、EMC / 安规、工厂产线测试与老化、HDCP 与色彩校准、
   真实的背光驱动与调光闪烁测试。

> 之所以把范围切成"与硬件无关的协议 / 状态机 / 算法层"，是因为这一层**最容易写错、也最值得单测**：
> 换一个 scaler 平台，下面这些东西的逻辑一行都不用改。

---

## 为什么做这个

一台显示器上电之后，固件要连续处理这几件事，而每一件都有"看着能跑、其实错了"的坑：

| 环节 | 现场真正难的地方 | 本仓库怎么处理 |
|---|---|---|
| **EDID** | 主机靠 EDID 决定输出什么分辨率；字节序、5-bit 厂商码、DTD 里的消隐与同步偏移、checksum 错一位就是"黑屏" | 完整的 base block 解析 + checksum + DTD→像素时钟/刷新率推导 + 与期望模式列表对账 |
| **DDC/CI** | 主机调亮度的通道；报文长度、XOR 校验、VCP 是表格值还是连续值、超时重试 | 主机侧与从机侧都实现，带 VCP 表、clamp、错误码与重试统计 |
| **输入源切换** | 热插拔会抖、无信号要自动找源、切源失败不能黑屏、用户连按不能来回切 | 状态机 + HPD 去抖 + 最短驻留 + 超时回退到上一个有效源 |
| **OSD** | 菜单树导航、无操作超时退出、**工厂模式**的进入序列不能被误触发 | 菜单树状态机 + 空闲超时 + 工厂模式"正例通过 / 反例不进入" |
| **背光 PWM** | 占空比线性 ≠ 眼睛线性；占空比 0 是熄灭不是最暗；直接跳变会闪；面板过热要降额 | gamma 编码 LUT + 最小占空比下限 + 渐变（可测"每步多小"）+ 两级过温降额 |

---

## 快速开始

```bash
make          # 构建端到端仿真程序
make test     # 构建并运行自检（最后一行是 "<N> checks passed"）
make sim      # 跑端到端仿真，结果写到 results/
```

只依赖 C99 编译器和 libm。CI 上验证过 **gcc 与 clang、Linux 与 macOS**，
并额外跑一条 **ASan + UBSan** 作业。

```bash
# 严格编译（CI 用的就是这一行）
make test CFLAGS="-std=c99 -O2 -Wall -Wextra -Wpedantic -Werror -ffp-contract=off"
```

---

## 目录结构

```
src/dfw.h         公开接口（五个模块 + 仿真入口）
src/edid.c        EDID 1.4 base block：解析、校验、DTD 定时推导、模式对账
src/ddc_ci.c      DDC/CI：报文层、VCP 表、主机侧与从机侧、重试策略
src/source_mux.c  输入源切换状态机：HPD 去抖、最短驻留、搜索与超时回退
src/osd.c         OSD 菜单树状态机：导航、空闲超时、工厂模式
src/backlight.c   背光 PWM：gamma LUT、最小亮度、渐变、过温降额
src/main_sim.c    端到端仿真（上电 → EDID → 选源 → OSD → 背光 → DDC 读回）
test/test_dfw.c   自检测试套件
tools/audit_api.py 检查 dfw.h 里声明的函数是否都有定义（防止"声明了没实现"）
```

---

## 实测结果

以下全部由 `make sim` 真实跑出，原始输出在 `results/metrics.txt`，曲线在 `results/backlight_lut.csv`。
**没有一个是写死的常量表。**

### 1. EDID

```
parse status            : 0 (ok)
manufacturer            : AOC (0x05E3)      product code 0x242A
EDID version            : 1.4               digital, 10 bpc, interface code 5 (DisplayPort)
screen size             : 520x290 mm (23.4 inch diagonal)
gamma                   : 2.20
DTDs found              : 2
  mode 1                : 1920x1080 @ 60.00 Hz  (pclk 148.50 MHz, h_total 2200, v_total 1125)
  mode 2                : 1280x720  @ 60.00 Hz  (pclk  74.25 MHz, h_total 1650, v_total  750)
range limits            : v 48..75 Hz, h 30..83 kHz, max pclk 170 MHz
expected modes          : 2
corrupted byte 100      : parse=-4 (checksum mismatch)
```

**刷新率是从 DTD 反算出来的**（`pclk / (h_total × v_total)`），不是查表：148.50 MHz ÷ (2200 × 1125) = **60.00 Hz**。

### 2. DDC/CI

```
VCP table               : 12 controls
GET 0x10 brightness     : value=50 max=100
SET 0x10=42 then GET    : value=42 (read back OK)
SET 0x10=250 (over max) : value=100 (clamped to max)
GET 0x7A (unsupported)  : result=0x01
one silent attempt      : retries used=1, recovered=yes
```

**"Set 之后 Get 能读回"是这一层的核心契约**，也是本仓库抓出最重要 bug 的地方（见下）。

### 3. 输入源切换

```
10 HPD toggles in 100 ms: current HDMI1 -> HDMI1, extra switch attempts=0
HDMI1 held low >120 ms  : present=no
select dead VGA         : current DP -> DP, showing an absent input=no
```

**100 ms 内抖 10 次热插拔，一次多余的切源都没有**（去抖生效）；
**切到没插线的 VGA 不会停在黑屏**，而是留在原来的 DP 上。

### 4. OSD

```
items                   : 6            idle timeout: 15000 ms
after 30 s idle         : state=closed
factory sequence (correct order)  : entered=yes
factory sequence (wrong order)    : entered=no
```

工厂模式**只有按对顺序（关机态长按 OK 5 秒 → 松开后在 2 秒窗口内按 UP）才进得去**，
顺序错或按得太短都进不去。

### 5. 背光 PWM

```
PWM                     : 4095 counts @ 20000 Hz -> period 50.0 us, one step = 0.0244 %
gamma 2.20, floor 1.00 %
LUT monotonic           : yes
brightness 0/25/50/75/100 -> counts 41/194/891/2175/4095 (of 4095)
gamma notice            : 50 % on the bar is only 891 counts (21.8 % duty)
fade 100% -> 10%        : 4095 -> 41 counts in 30 ticks (300 ms),
                          30 visible steps, largest step 136 counts
panel 25/40/55/70/85 C  : output 100/100/100/70/40 %
derate events           : 4
cooled back to 25 C     : output 100 % (user asked 100 %)
```

两件事最能说明"为什么不能直接写占空比"：

* **用户看到的 50 % 只对应 21.8 % 的占空比** —— 这就是 gamma 编码的意义；
* **从 100 % 渐变到 10 % 走了 30 个可见步、最大一步 136 counts**（满量程 4095），
  而不是一次跳变 —— 直接写寄存器会在面板上闪一下，也会冲击驱动电路。

### 6. 测试

```
340 checks passed
```

CI 上跑 5 条作业：ubuntu / macos × gcc / clang（全部 `-Werror`）+ 一条 ASan/UBSan。

---

## 开发过程中被抓出来的真问题

这些都不是"设计时想到的"，是**测试和仿真逼出来的**。改法都在提交历史里。

### 1. EDID 描述符整体偏移了两个字节（最严重）

解析器把描述符 tag 放在**第 5 字节**、payload 放在第 6 字节起；
而 VESA 的 18 字节描述符是 **tag 在第 3 字节、payload 在第 5 字节起**。

问题在于：**解析器和它的构造函数错得一模一样**，所以自己写、自己读完全正常 ——
只有测试**按规范手工拼一个描述符**（而不是用库自己的 builder）才暴露出来。
现实后果：**这个解析器读不懂任何一台真实显示器的 EDID**（读不到产品名、序列号与范围限制）。

**修法**：解析与构造同时改到规范布局，并在测试里保留"手工按规范拼字节"的用例。

### 2. Get VCP 只发了 1 字节的 VCP 码

DDC/CI 的 `Get VCP Feature Request` 是 `01 <vcp_hi> <vcp_lo>` —— **VCP 码是两个字节**。
主机侧只发了低字节，于是帧长变成 6 字节，从机把 **XOR 校验字节当成了低字节**，
回出来的 VCP 码是 `0x102A`，主机校验不过直接报 `BAD_OPCODE`。

后果：**所有高字节非零的 VCP（MCCS 里 0xD0 那一整段）全部读不到**；
而 0x10 亮度"偶尔能读到"只是因为它刚好落在一个能被误解析的位置上。

**修法**：主机发 3 字节 payload（帧长 7），从机的长度检查同步改成 `req_len != 7`。

### 3. 屏幕尺寸单位搞错了（cm 当成 mm）

EDID base block 的第 21/22 字节是**厘米**，而 DTD 里的尺寸是**毫米** —— 同一个规范里两种单位。
解析器按毫米处理，于是一台 52 × 29 cm 的显示器被算成 **2.3 英寸**。

**修法**：base block 的字节 ×10 存成毫米（字段名 `image_w_mm` 才是诚实的），
DTD 侧不动；顺带修了"只有一个尺寸时"的宽高比分支 —— 它必须用**原始字节**算 `(v+99)/100`，
不能用换算后的毫米值。

### 4. `last_delta_counts` 在渐变结束时报的是全程差

`last_delta_counts` 的用途是让测试证明"渐变是逐步的"，但它在最后一拍把
**整段差值**写进去，于是"最大单步"永远等于总量，这个指标就废了。

**修法**：最后一拍只报**这一拍**移动了多少。

### 5. 只在 macOS 上挂的浮点断言

clang（以及较新的 gcc）会把 `a*b+c` 融合成一条 FMA，浮点末位变化，
于是 EDID 的定时换算与背光 LUT 的断言**只在 macOS 上**失败。
**修法**：所有编译命令加 `-ffp-contract=off`（Makefile 与 CI 里都有，且写了原因）。

### 6. `make sim | head` 会让 CI 非 0 退出

管道提前关闭会让程序吃到 SIGPIPE，被信号杀掉而不是执行失败。
**修法**：CI 里先重定向到文件再 `tail`。

---

## 怎么用它复习 / 面试怎么讲

* **一条命令看全貌**：`make sim` 然后读 `results/metrics.txt`。
* **想看某个机制**：`src/` 里每个文件顶部都写了协议格式与公式的来龙去脉。
* **面试可以讲的三件事**：
  1. **"Set 之后 Get 读不回来"** —— 怎么从"回出来的 VCP 码是 0x102A"反推出
     "帧长 6 字节导致从机把校验当成了低字节"，以及为什么**自己写自己读的测试永远抓不到它**；
  2. **gamma 编码的实证** —— 用户条上的 50 % 只对应 21.8 % 的占空比，为什么不能直接线性映射；
  3. **去抖的可测化** —— "100 ms 内抖 10 次热插拔，额外切源次数 = 0"是一个可以写进测试的判据，
     比"我做了去抖"有力得多。

---

## License

MIT，见 `LICENSE`。
