# Vitis Vision SGBM 双目立体匹配全流程分析

> 分析对象：本仓库 `Vitis_Libraries/vision` 子模块
> 源码仓库版本：`Vitis_Libraries` HEAD `629b2c979f65561f07e4e87b860f306cb480895e`
> 分析日期：2026-07-19
> 结论依据：本仓库 L1/L2 示例、`xf_sgbm.hpp` 实现及仓库自带 API 文档。仓库中没有本地生成的 csynth/implementation 报告，因此本文把“源码事实”“官方文档估计”和“按源码推导值”明确区分。

## 目录

- [1. 结论摘要](#1-结论摘要)
- [2. 分析范围与两套示例的关系](#2-分析范围与两套示例的关系)
- [3. 端到端 PS/PL 数据流](#3-端到端-pspl-数据流)
- [4. 主要步骤、函数、计算位置与并行度](#4-主要步骤函数计算位置与并行度)
  - [4.1 Census 变换](#41-census-变换)
  - [4.2 Hamming 初始代价](#42-hamming-初始代价)
  - [4.3 四路径 SGM 递推和聚合](#43-四路径-sgm-递推和聚合)
  - [4.4 WTA 视差选择](#44-wta-视差选择)
- [5. 当前参数及其含义](#5-当前参数及其含义)
- [6. 流水线吞吐与延迟模型](#6-流水线吞吐与延迟模型)
  - [6.1 1080p 性能](#61-1080p-性能)
  - [6.2 L1 720p 估计](#62-l1-720p-估计)
- [7. 存储、带宽和资源分析](#7-存储带宽和资源分析)
  - [7.1 PL 避免完整代价体](#71-pl-避免完整代价体)
  - [7.2 主要片上状态](#72-主要片上状态)
  - [7.3 官方资源估计](#73-官方资源估计)
  - [7.4 外存流量](#74-外存流量)
- [8. 主要瓶颈排序](#8-主要瓶颈排序)
  - [8.1 第一瓶颈：路径递推的数据依赖与 `II=2`](#81-第一瓶颈路径递推的数据依赖与-ii2)
  - [8.2 第二瓶颈：BRAM banking 和片上状态规模](#82-第二瓶颈bram-banking-和片上状态规模)
  - [8.3 第三瓶颈：高 PU 下的组合逻辑、DSP 与布线](#83-第三瓶颈高-pu-下的组合逻辑dsp-与布线)
  - [8.4 系统瓶颈：当前 host 没有帧间重叠](#84-系统瓶颈当前-host-没有帧间重叠)
  - [8.5 验证瓶颈：CPU reference 的时间和内存](#85-验证瓶颈cpu-reference-的时间和内存)
- [9. 功能边界：该实现有与没有什么](#9-功能边界该实现有与没有什么)
  - [9.1 已实现](#91-已实现)
  - [9.2 未实现](#92-未实现)
- [10. 正确性与集成风险](#10-正确性与集成风险)
- [11. 参数调整的性能/资源趋势](#11-参数调整的性能资源趋势)
- [12. 源码证据索引](#12-源码证据索引)
- [13. 最终判断](#13-最终判断)

## 1. 结论摘要

1. 本仓库 SGBM 的系统级入口是 L2 示例的 `main()`，硬件核入口是 `sgbm_accel()`；L1 示例主要用于 C 仿真、HLS 综合和 RTL 协同仿真，硬件核名为 `semiglobalbm_accel()`。两者最终调用同一个 `xf::cv::SemiGlobalBM()` 实现。
2. 在 VCK190 等 SoC 平台上，图像文件读取、XRT/OpenCL 调度、DDR 缓冲区管理、结果保存和参考结果校验运行在 PS；Census、匹配代价、4 路径 SGM 聚合和 WTA 视差选择运行在 PL。若使用 U200，所谓“PS 部分”实际运行在外部 x86 主机，并不存在片上 PS。
3. 当前 L2 配置固定为 1920×1080 上限、8 位单通道、5×5 Census、64 个整数视差、32 个视差并行单元、4 条路径、`P1=20`、`P2=40`、`NPPC1`，输出是 8 位整数视差索引。
4. PL 内部采用 DATAFLOW 流水，不保存完整的 `H×W×D` 代价体；主要片上状态是路径历史 `Lr[R-1][D][COLS]`。默认参数下，仅 `Lr` 和 `Lr_min` 的原始数据量约 365.625 KiB，且按路径和视差完全分区，BRAM 是主要资源压力。
5. 稳态吞吐的首要瓶颈是 `xFSGBMoptimization()`：`D/PU=2` 个视差批次，每批目标 `II=2`，因此约 4 cycle/pixel。1080p 的主循环约 8,294,400 cycle；在 200 MHz 下为 41.472 ms，与仓库 API 文档给出的 42 ms 性能估计吻合。
6. 这不是 OpenCV `StereoSGBM` 的全部功能：代码没有双目标定/校正、左右一致性检查、唯一性检查、亚像素插值、speckle/中值/WLS 滤波、无效视差标记或深度换算。输入必须预先极线校正。
7. 示例的正确性检查存在缺口：它只统计 `硬件视差 - 参考视差 > 0` 的像素，硬件结果小于参考值时不会报错，因而“Test Pass”不能证明逐像素完全一致。

## 2. 分析范围与两套示例的关系

| 层级 | 入口 | 最大尺寸 | 运行目的 | 主机/器件配置 |
|---|---|---:|---|---|
| L1 | `semiglobalbm_accel()` | 1280×720 | C 仿真、HLS 综合、cosim | HLS 时钟约束 3.3 ns；默认平台 VCK190，旧 `run_hls.tcl` 仍保留 U200 part |
| L2 | `main()` → `sgbm_accel()` | 1920×1080 | 完整 XRT/OpenCL 系统运行 | VCK190 为 AArch64 PS host，U200 为 x86 host；VCK190 请求 100 MHz，其他允许平台请求 300 MHz |
| 算法库 L1 include | `xf::cv::SemiGlobalBM()` | 由模板 `ROWS/COLS` 决定 | 两套 wrapper 共同使用的 PL 算法实现 | 固定 `NPPC1`、5×5 Census，支持 `R=2/3/4` |

当前仓库没有 SGBM 的 L3 `stereopipeline`；`vision/L3/examples/stereopipeline` 调用的是 `StereoBM`，不能当作 SGBM 的前后处理流水线。下文 Census、cost、optimization、WTA 等函数是同一个 `sgbm_accel` 内部的 HLS dataflow 进程/逻辑层次，不是四个可由 PS 分别启动的独立 kernel。

主要源码：

- L2 主机端：`vision/L2/examples/sgbm/xf_sgbm_tb.cpp`
- L2 PL wrapper：`vision/L2/examples/sgbm/xf_sgbm_accel.cpp`
- L2 当前参数：`vision/L2/examples/sgbm/config/xf_config_params.h`
- L2 构建与频率：`vision/L2/examples/sgbm/Makefile`
- 核心算法：`vision/L1/include/imgproc/xf_sgbm.hpp`
- 官方 API、资源与性能估计：`vision/docs/src/api-reference.rst:16728-16850`

以下“PS”按 VCK190 这类嵌入式平台表述；对 U200，应把 PS 理解为外部 x86 host。L1 testbench 则是开发机上的仿真程序，不等同于板上 PS 软件。

## 3. 端到端 PS/PL 数据流

```text
PS / Host
  读取左右灰度图 cv::imread(..., 0)
  → 创建 XRT/OpenCL context、加载 krnl_sgbm.xclbin
  → 分配左右输入和视差输出 device buffer
  → 两次阻塞式 H2D 写入
                         │
                         ▼
PL: sgbm_accel
  gmem0/gmem1 读取与 32-bit→XF_8UC1 解包
  → 左右 5×5 Census（两路并行）
  → 64 视差 Hamming 代价（32 视差/批，共 2 批）
  → 4 路径 SGM 动态规划与聚合（32 视差×4 路展开，II=2）
  → WTA 最小代价视差（32 路最小树，共 2 批）
  → XF_8UC1→32-bit 打包并写 gmem2
                         │
                         ▼
PS / Host
  阻塞式 D2H 读回
  → 保存 hls_out.png
  → 运行纯 CPU 参考 SGM、保存 disp_map.png
  → 比较并保存 diff.png
```

kernel 的三个 `m_axi` 端口分别位于 `gmem0/gmem1/gmem2` bundle，可形成独立 AXI master 端口；但示例没有 `sp=` 等显式 memory-bank 映射，最终是否落到独立物理 DDR bank/NoC 通道取决于平台和链接器。

## 4. 主要步骤、函数、计算位置与并行度

| 序号 | 主要步骤 | 对应函数/代码 | 位置 | 当前并行度与流水特征 | 主要限制或瓶颈 |
|---:|---|---|---|---|---|
| 1 | 读取输入 | `main()` 中 `cv::imread(argv[1/2], 0)` | PS/Host | CPU 图像解码；两幅图依次读取 | 文件 I/O 和解码不计入 kernel latency；没有校正，也没有检查左右尺寸是否相同 |
| 2 | 初始化运行时 | `xcl::get_xil_devices()`、`cl::Context`、`cl::Program`、`cl::Kernel` | PS/Host | 单 kernel、单 command queue | xclbin 加载和初始化是一次性开销，不应混入单帧 PL 算法性能 |
| 3 | 分配/上传图像 | `cl::Buffer`、两次 `enqueueWriteBuffer(..., CL_TRUE, ...)` | PS/Host ↔ device memory | 左、右各 1 byte/pixel；当前调用为阻塞且串行 | 没有 ping-pong、多帧排队或 H2D/计算重叠；VCK190 主要经过片上 DDR/NoC 路径，U200 还涉及 PCIe |
| 4 | 外存到 `xf::cv::Mat` | `Array2xfMat()` | PL | 左右端口各 32 bit，内部格式 `XF_8UC1/NPPC1`；顶层 DATAFLOW | 32-bit 总线宽度不等于 4 pixel/clock 算法吞吐，核心仍固定 NPPC1；实际带宽受存储映射影响 |
| 5 | Mat 转流 | `SemiGlobalBM()` 内部 `_src_mat_l/r.read()` 循环 | PL | 左右像素同一循环读取，目标 1 pixel/cycle | 通常不是默认配置瓶颈 |
| 6 | 左右 Census | `xFCensusTransformKernel()` → `xFCensus5x5()` → `xFProcessCensusTransform5x5()` → `xFComputeTransform5x5()` | PL | 左、右两套 Census 在 DATAFLOW 中并行；5 行缓冲；单图每像素 24 次比较完全展开；列循环 pipeline | 需要先填充行缓存；仅支持 5×5、常数 0 边界、NPPC1；行缓存随 `COLS` 增长 |
| 7 | 32→24 bit Census 流转换 | `SemiGlobalBM()` 中 `_src_census24_l/r.write(...)` | PL | 左右同步，目标 1 pixel/cycle | 只是格式转换，不是主计算瓶颈 |
| 8 | 初始匹配代价 | `xFSGBMcomputecost()` | PL | `PU=32` 路完全展开；每批并行算 32 个 24-bit Hamming distance；`D/PU=2` 批，批循环 `II=1` | 约 2 cycle/pixel；增加 `D` 线性增时，增加 `PU` 会增加 XOR/popcount 和布线资源 |
| 9 | 路径递推与代价聚合 | `xFSGBMoptimization()`、`xFMinSAD<4>::find()`、`xFMinSAD<PU>::find()` | PL | `PU=32` 与 `R=4` 两层完全展开，即每批形成 32×4 路递推；批循环目标 `II=2` | **默认稳态主瓶颈**：约 4 cycle/pixel；路径数据依赖、BRAM 多 bank 访问、最小值树和大规模展开共同限制 II、频率与布局布线 |
| 10 | Winner-Takes-All | `xfSGBMcomputedisparity()`、`xFMinSAD<PU>::find()` | PL | 每批对 32 个 16-bit 聚合代价做平衡最小树，再比较 2 个批次最小值；批循环 `II=1` | 约 2 cycle/pixel；PU 越大，比较树和扇出越大 |
| 11 | 输出打包/写 DDR | `xfMat2Array()` | PL | `XF_8UC1/NPPC1` 内部流打包为 32-bit `m_axi`，独立 `gmem2` | 与前级 DATAFLOW；物理 memory bank 未显式绑定 |
| 12 | 读回输出 | `enqueueReadBuffer(..., CL_TRUE, ...)` | PS/Host ↔ device memory | 1 byte/pixel，阻塞读回 | 不包含在示例打印的 hardware-function latency 中 |
| 13 | CPU 参考结果 | `compute_SGM()` 及其子函数 | PS/Host | 纯串行 C++ 多重循环，完整分配 `H×W×D` 和 `R×H×W×D` 数组 | 只应用于验证，时间和内存远大于 PL 核；不能放入生产逐帧路径 |
| 14 | 比较与保存 | `saveDisparityMap()`、差值循环、`cv::imwrite()` | PS/Host | 串行 CPU | 当前比较只检查正差，存在假通过风险 |

### 4.1 Census 变换

对中心像素与 5×5 邻域内其余 24 个像素逐一比较，生成 24-bit 描述子：

```text
bit_k = 1, if neighbor_k < center; otherwise 0
```

源码中的 5×5 局部窗 `src_buf[5][5]` 完全分区，两个嵌套邻域循环均 `UNROLL`。`buf[5][COLS]` 是五行循环行缓存，绑定到 BRAM；左右图调用各自独立，能够并行工作。图像边界使用常数 0。

### 4.2 Hamming 初始代价

左像素 `(r,c)` 的描述子与右图 `(r,c-d)` 的描述子异或并做 24-bit popcount：

```text
C(p,d) = popcount(census_left(r,c) XOR census_right(r,c-d))
```

`r_buff[NDISP]` 保存当前行右图最近的 Census 描述子并完全分区；`c-d<0` 的区域以 0 参与匹配。默认 `D=64, PU=32`，所以每个像素分两批，每批同时生成 32 个 8-bit cost，写入 32 路 `_cost[]` stream。

### 4.3 四路径 SGM 递推和聚合

实现采用标准归一化递推：

```text
Lr(p,d) = C(p,d)
          + min(Lr(p-r,d),
                Lr(p-r,d-1) + P1,
                Lr(p-r,d+1) + P1,
                min_k Lr(p-r,k) + P2)
          - min_k Lr(p-r,k)

S(p,d) = sum_r Lr(p,d)
```

当前 `R=4` 的前驱方向是：

| `r` | 前驱坐标 | 扫描方向含义 |
|---:|---|---|
| 0 | `(row, col-1)` | 左 → 右 |
| 1 | `(row-1, col-1)` | 左上 → 右下 |
| 2 | `(row-1, col)` | 上 → 下 |
| 3 | `(row-1, col+1)` | 右上 → 左下 |

四条路径都能在单次从上到下、从左到右的 raster scan 中得到，因此没有反向帧扫描。它是经典 8 路/16 路 SGM 的近似，仓库 API 文档也明确说明只考虑四个方向。

对每批视差，`loop_pu` 和 `loop_directions` 都完全展开。默认情况下硬件同时形成 128 个路径状态的递推逻辑，再把同一视差的 4 路 `Lr` 相加成 16-bit `_agg_cost`。由于路径状态有前后像素依赖，外层视差批循环明确约束为 `II=2`。

### 4.4 WTA 视差选择

`xfSGBMcomputedisparity()` 在 64 个聚合代价中选择最小值对应的视差：

```text
d_out(p) = argmin_d S(p,d)
```

`xFMinSAD<32>` 采用递归二分的平衡比较树；两个 32 视差批次分别产生局部最小值，再比较全局最小值。结果直接输出 `uint8` 整数视差 0～63，没有亚像素拟合。

## 5. 当前参数及其含义

| 参数 | L1 示例 | L2 示例 | 含义/约束 |
|---|---:|---:|---|
| `HEIGHT × WIDTH` | 720×1280 | 1080×1920 | 编译时最大尺寸；运行时 `rows/cols` 不得超过它 |
| `IN_TYPE` | `XF_8UC1` | `XF_8UC1` | 8-bit 单通道输入 |
| `OUT_TYPE` | `XF_8UC1` | `XF_8UC1` | 8-bit 整数视差输出 |
| `WINDOW_SIZE` | 5 | 5 | 只支持 5×5 Census |
| `TOTAL_DISPARITY/NDISP` | 64 | 64 | 视差搜索为 0～63；实现支持 2～256 |
| `PARALLEL_UNITS/PU` | 32 | 32 | 同时计算的视差数；必须 `D≥PU` 且 `D%PU=0` |
| `NUM_DIR/R` | 4 | 4 | 聚合路径数；实现只接受 2、3 或 4 |
| `SMALL_PENALTY/P1` | 20 | 20 | host 默认值；以 8-bit AXI-Lite 标量在运行时传入，小视差变化惩罚 |
| `LARGE_PENALTY/P2` | 40 | 40 | host 默认值；以 8-bit AXI-Lite 标量在运行时传入，要求 `P1<P2≤100` |
| `NPPCX` | `XF_NPPC1` | `XF_NPPC1` | 每拍一个像素的接口/算子模式，固定不可改为 NPPC8 |
| `INPUT_PTR_WIDTH` | 32 | 32 | 外部输入 AXI 指针宽度 |
| `OUTPUT_PTR_WIDTH` | 32 | 32 | 外部输出 AXI 指针宽度 |
| `XF_CV_DEPTH_*` | 2 | 2 | wrapper 中 `xf::cv::Mat` stream 深度 |

注意：`NPPC1` 或官方表格中的 “1 pixel/clock operating mode” 表示像素并行接口模式，不代表完整 SGBM 核必然每周期输出一个像素。默认聚合级的实际稳态限制约为 4 cycle/pixel。

## 6. 流水线吞吐与延迟模型

令：

- `H, W`：实际图像高、宽；
- `D`：总视差数；
- `PU`：并行视差单元数；
- `Q=D/PU`：每像素的视差批次数；
- `f`：PL kernel 实际实现后的时钟频率。

从源码 pragma 可得到稳态近似：

| PL 阶段 | 批循环 II | 每像素批次数 | 稳态周期/像素近似 |
|---|---:|---:|---:|
| Census | 约 1 | 1 | 约 1 |
| Hamming cost | 1 | `Q` | `Q` |
| SGM optimization | 2 | `Q` | **`2Q`** |
| WTA disparity | 1 | `Q` | `Q` |
| 输入/输出流 | 约 1 | 1 | 约 1 |

由于这些阶段由 DATAFLOW 重叠，稳态吞吐由最慢阶段而不是各阶段延迟之和决定：

```text
cycles_per_pixel ≈ max(1, Q, 2Q, Q) = 2D/PU
steady_frame_cycles ≈ H × W × 2D/PU
steady_frame_time ≈ H × W × 2D / (PU × f)
```

默认 `D=64, PU=32`，所以约为 4 cycle/pixel。

### 6.1 1080p 性能

`1920×1080=2,073,600` pixel，主循环约：

```text
2,073,600 × 4 = 8,294,400 cycle/frame
```

| 频率 | 数字来源 | 仅稳态主循环的推导下限 | 对应上限帧率 | 说明 |
|---:|---|---:|---:|---|
| 100 MHz | L2 VCK190 Makefile 请求频率 | 82.944 ms | 12.06 fps | 未加初始化、填充/排空、DDR 和 XRT 开销 |
| 200 MHz | 仓库 API 文档估计条件 | 41.472 ms | 24.11 fps | 官方文档报告约 42 ms，与推导吻合 |
| 300 MHz | L2 非 VCK190 分支请求频率 | 27.648 ms | 36.17 fps | 只是请求频率；必须以实际 timing closure 和实测为准 |

仓库自带 API 文档对 1080p、64 视差、32 PU、200 MHz 给出的性能估计是 **42 ms**，不是本文实测值。示例程序打印的 `Latency for hardware function` 来自 `enqueueTask` event 的 start/end，只覆盖 kernel task，不覆盖 H2D、D2H、文件 I/O、参考计算和结果保存。

除稳态主循环外，实际 kernel 还有：

- Census 五行缓存的启动与 DATAFLOW 填充/排空；
- `xFSGBMoptimization()` 在每次调用开始时对 `Lr`、`Lr_r0` 和 `Lr_min` 的清零；其中 `Lr` 清零循环按编译时 `COLS` 而不是运行时 `width` 运行；
- AXI burst、DDR/NoC 仲裁和可能的 memory-bank 冲突；
- kernel 启动和 runtime 调度开销。

因此上表只能作为结构性下限/容量估计，不能代替板上测量。

### 6.2 L1 720p 估计

L1 的 3.3 ns HLS 约束相当于约 303 MHz；若该频率实际实现成功，1280×720 的稳态主循环约 3,686,400 cycle，即约 12.17 ms（82.2 fps）。这同样只是源码吞吐模型，不是综合后或板上实测。

## 7. 存储、带宽和资源分析

### 7.1 PL 避免完整代价体

PL 通过 stream 在 cost、optimization 和 WTA 之间传递数据，没有落地完整 `H×W×D` 代价体。1080p、64 视差时，一个 8-bit cost volume 本身就有约 126.56 MiB；若聚合结果按 16-bit 保存则约 253.13 MiB。流式实现显著降低了外存带宽和容量需求。

### 7.2 主要片上状态

`xFSGBMoptimization()` 中最主要的状态是：

```text
Lr[R-1][D][COLS]       : (4-1)×64×1920 = 368,640 byte = 360 KiB
Lr_min[R-1][COLS]      : (4-1)×1920    =   5,760 byte = 5.625 KiB
合计原始数据量                              = 365.625 KiB
```

`Lr` 在路径维和视差维都 `ARRAY_PARTITION complete`，相当于 3×64 个按列寻址的独立 bank。这样才能并行访问 32 个视差和 4 条路径，但会产生大量小 BRAM bank、容量碎片以及很高的布线复杂度。BRAM 开销近似随 `(R-1)×D×COLS` 增长，而不是随图像高度增长。

左右 Census 还各有 `5×COLS` 的 8-bit 行缓存，L2 下两路合计原始数据约 18.75 KiB。`r_buff[D]` 完全分区，主要实现成寄存器。

### 7.3 官方资源估计

仓库 API 文档对 1920×1080、64 视差、32 PU、5×5、200 MHz 给出：

| BRAM_18K | DSP48E | FF | LUT |
|---:|---:|---:|---:|
| 205 | 141 | 11,856 | 19,102 |

这是仓库文档中的参考估计，不是当前目录下重新综合得到的结果；文档表格没有在该行再次注明 `R`，而当前示例配置为 `R=4`。移植到不同器件、Vitis 版本、频率约束或参数后必须重新综合确认。

### 7.4 外存流量

单帧 1080p 的 kernel 必需净数据量为：

```text
左图 2,073,600 byte + 右图 2,073,600 byte + 输出 2,073,600 byte
= 6,220,800 byte ≈ 5.93 MiB/frame
```

默认算法核每像素约需 4 个时钟，因此从纯净吞吐量看，外存带宽通常不像路径聚合那样先成为瓶颈；但是当前系统端没有显式 bank 绑定，且 PS/Host 使用两次串行阻塞上传和一次阻塞下载，端到端多帧吞吐仍需用 XRT profile 实测。

## 8. 主要瓶颈排序

### 8.1 第一瓶颈：路径递推的数据依赖与 `II=2`

`xFSGBMoptimization()` 需要同一路径前驱像素的 `Lr`、邻近视差 `d-1/d/d+1` 以及该前驱像素全部视差的最小值。数据依赖使其不能像 Census 那样简单做到每拍一个像素。默认两批视差且每批 `II=2`，形成 4 cycle/pixel 的全流水瓶颈。

### 8.2 第二瓶颈：BRAM banking 和片上状态规模

为了 32 视差并行，`Lr` 的视差维完全分区。增大视差范围 `D` 会同时增加运行周期和 BRAM bank；增大图像宽度 `COLS` 会增加每个 bank 的深度；增大路径数 `R` 会近似成比例增加状态。当前支持的最大路径数也只有 4。

### 8.3 第三瓶颈：高 PU 下的组合逻辑、DSP 与布线

每个视差/路径都包含四选一最小值、加减、路径聚合，并且还要计算每条路径的 32 路最小树；Hamming 和最终 WTA 也各有 32 路逻辑。`PU` 增大能减少批次数，却会加大比较树、DSP/加法器数量、扇出和跨 bank 布线，可能降低可实现频率。请求 300 MHz 不等于实现后一定能达到 300 MHz。

### 8.4 系统瓶颈：当前 host 没有帧间重叠

两次输入写、kernel 和输出读都是顺序阻塞流程，且 command queue 没有启用 out-of-order。单帧演示正确，但不适合直接代表视频流水性能。生产设计应至少考虑预分配 buffer、双/多缓冲、异步迁移和事件依赖；在嵌入式平台还应优先减少 PS 侧无必要的整帧复制。

### 8.5 验证瓶颈：CPU reference 的时间和内存

1080p 默认参数下，CPU reference 同时持有：

- `accumulatedCost`：约 506.25 MiB；
- `Lr`（4 路）：约 2,025 MiB；
- `aggregatedCost`：约 506.25 MiB；
- 两幅 64-bit `long int` Census：约 31.64 MiB。

仅这些数组就约 3.0 GiB，且计算复杂度为 `O(H×W×D×R)` 的串行多重循环。在 VCK190 PS 上它可能成为绝对主耗时甚至触发内存不足，但这不代表 PL SGBM 核的性能。

## 9. 功能边界：该实现有与没有什么

### 9.1 已实现

- 预校正双目灰度图的水平视差搜索；
- 5×5 Census 描述子；
- 24-bit Hamming 初始匹配代价；
- 2、3 或 4 路径的 SGM 代价递推，示例使用 4 路；
- 全图统一的运行时标量 `P1/P2` 平滑惩罚；
- WTA 整数视差输出。

### 9.2 未实现

- 相机采集、同步和曝光控制；
- 畸变校正和双目极线校正；
- 自适应 `P2` 或基于图像梯度的惩罚；
- 右视差图与左右一致性检查；
- uniqueness ratio、遮挡检测和显式 invalid disparity；
- 亚像素抛物线插值；CPU reference 中相关代码也被注释；另有一个空洞填补式 `interpolateDisp()` 函数，但主流程从未调用；
- speckle 去除、中值/WLS 等后处理；
- 由 `Z=fB/d` 或 Q 矩阵进行的深度/三维坐标换算。

所以若目标是“相机输入到深度图”的完整双目产品流水线，仍需在 SGBM 前后补上：

```text
采集/同步 → 去畸变与极线校正 → 灰度化 → 本 SGBM → 置信度/一致性/滤波 → 视差转深度
```

这些步骤在本 SGBM 示例中没有对应 PL kernel；直接使用示例时只能由 PS/Host 或其他自建 PL 模块承担。

## 10. 正确性与集成风险

1. **输入必须已校正。** cost 只比较左 `(r,c)` 和右 `(r,c-d)`，没有纵向搜索；未校正图像会直接破坏匹配。
2. **左右尺寸必须相同且不超过编译上限。** host 只用左图尺寸设置 buffer 和 kernel 参数，没有显式检查右图尺寸，也没有在硬件运行时可靠保护超限输入。
3. **输出是裸整数视差。** 0 既可能是真实零视差，也可能是无匹配结果；代码没有额外有效位或 invalid 值。
4. **当前 test 的差分判断不对称。** `d_val = hls_out - reference` 后只在 `d_val>0` 时计错；应使用 `d_val!=0` 或绝对差阈值，否则负差被漏检。
5. **示例没有核对左右图时间同步、校准参数或 baseline/focal length。** 它不能单独保证深度准确度。
6. **官方 42 ms 和资源表是估计，不是当前板卡实测。** 最终结论应来自目标平台的 csynth、place-and-route timing、XRT kernel profile 和端到端多帧测量。
7. **频率配置按平台不同。** L2 Makefile 对 VCK190 请求 100 MHz，对其他允许平台请求 300 MHz；`description.json` 中也有平台覆盖值。不能脱离具体平台笼统引用一个频率。

## 11. 参数调整的性能/资源趋势

| 调整 | 吞吐影响 | 资源/时序影响 | 精度影响 |
|---|---|---|---|
| 增大 `D` | 周期近似线性增加，除非同步增大 `PU` | `Lr` BRAM bank 近似线性增加 | 可覆盖更近物体/更大视差 |
| 增大 `PU` | `D/PU` 降低，吞吐提高 | Hamming、路径递推、最小树和布线显著增加，可能降低 Fmax | 数值算法不变 |
| 增大 `R` | 单批仍目标 II=2，但组合与状态更重，实际 Fmax/II 风险增大 | 路径状态、加减/比较和 BRAM 近似增加 | 通常更平滑、更接近全 SGM；本实现最多 4 路 |
| 增大 `COLS` | 每帧像素数增加 | `Lr` 和 Census 行缓存随宽度增加 | 不直接改变单像素算法 |
| 调整 `P1/P2` | 基本不改吞吐 | 基本不改结构 | 明显影响平滑度、边缘保持和错误匹配 |

若以性能为主要目标，不能只把 `PU` 从 32 改到 64；必须同时确认 BRAM、DSP/LUT、路由、实际 Fmax，以及 `xFSGBMoptimization()` 的依赖是否仍能达到目标 II。

## 12. 源码证据索引

| 结论 | 源码位置 |
|---|---|
| L2 默认分辨率、D/PU/R、P1/P2、类型与总线宽度 | `L2/examples/sgbm/config/xf_config_params.h:27-59` |
| 三个独立 m_axi bundle、DATAFLOW 和核心调用 | `L2/examples/sgbm/xf_sgbm_accel.cpp:21-60` |
| PS/Host 读图、buffer、上传、启动、计时与下载 | `L2/examples/sgbm/xf_sgbm_tb.cpp:348-469` |
| CPU reference 的 Census、Hamming、路径和 WTA | `L2/examples/sgbm/xf_sgbm_tb.cpp:21-335` |
| 不对称误差判断 | `L2/examples/sgbm/xf_sgbm_tb.cpp:489-512` |
| Census 24 比较完全展开与五行 BRAM 缓存 | `L1/include/imgproc/xf_sgbm.hpp:41-268` |
| 32 路 Hamming、右图移位缓存和 `II=1` | `L1/include/imgproc/xf_sgbm.hpp:375-458` |
| 路径状态、BRAM 分区、PU×R 展开和 `II=2` | `L1/include/imgproc/xf_sgbm.hpp:486-838` |
| WTA 最小值与视差输出 | `L1/include/imgproc/xf_sgbm.hpp:840-889` |
| 核心 DATAFLOW 调用顺序和参数断言 | `L1/include/imgproc/xf_sgbm.hpp:891-1005` |
| L2 平台、host 架构与请求频率 | `L2/examples/sgbm/Makefile:52-69,88-98,150-170` |
| 官方算法说明、资源与 42 ms 性能估计 | `docs/src/api-reference.rst:16728-16850` |

## 13. 最终判断

对当前 `D=64, PU=32, R=4` 配置，SGBM 核是一个以 **32 路视差并行 + 4 路路径并行 + 帧内 DATAFLOW** 为核心的 PL 流式实现。其计算性能主要由路径聚合的 `2×(D/PU)` cycle/pixel 决定，资源主要由按路径、视差分 bank 的 `Lr` 历史状态决定。默认 1080p@200 MHz 的约 42 ms 是有源码循环结构支持的合理估计，但 VCK190 示例实际请求的是 100 MHz，且端到端程序还有阻塞式数据搬运及极重的 CPU reference，不能把 42 ms 直接等同于示例总运行时间或产品帧率。

在产品级使用前，应以已校正的真实双目数据重新验证视差质量，修正 test 的漏检条件，并在目标板上分别测量 kernel-only、H2D/kernel/D2H 和包含前后处理的端到端帧率。
