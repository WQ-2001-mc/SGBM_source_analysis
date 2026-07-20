# Vitis Libraries 双目立体匹配源码结构分析（SGBM 与 SAD_BM）

> 分析对象：AMD/Xilinx `Vitis_Libraries` 中的双目立体匹配与深度相关开源代码
> 仓库版本：`v2026.1_re`，commit `629b2c979f65561f07e4e87b860f306cb480895e`  
> 文档整理日期：2026-07-13  
> 说明：本文以当前本地源码为准，并结合 AMD Vitis Vision 2026.1 官方文档说明 API 的设计目的、构建层级和正确使用方式。

## 目录索引

- [1. 结论先行](#section-1)
- [2. 双目深度的端到端数据流](#section-2)
- [3. L1、L2、L3 为什么都出现相似代码](#section-3)
- [4. 仓库内双目深度相关文件总览](#section-4)
- [5. SemiGlobalBM：算法原理与代码实现](#section-5)
  - [5.1 API 与模板参数](#section-5-1)
  - [5.2 算法调用链](#section-5-2)
  - [5.3 5×5 Census Transform](#section-5-3)
  - [5.4 Hamming 匹配代价](#section-5-4)
  - [5.5 SGM 路径递推与聚合](#section-5-5)
  - [5.6 Winner-Takes-All 输出](#section-5-6)
  - [5.7 参数约束](#section-5-7)
  - [5.8 默认示例配置](#section-5-8)
  - [5.9 官方资源与性能参考](#section-5-9)
  - [5.10 L1 HLS 封装与验证](#section-5-10)
  - [5.11 L2 kernel 与 Host](#section-5-11)
  - [5.12 SemiGlobalBM 的 FPGA 算子、输入输出与尺寸汇总](#section-5-12)
- [6. StereoBM：算法原理与代码实现](#section-6)
  - [6.1 API、源码位置与硬件定位](#section-6-1)
  - [6.2 核心调用链](#section-6-2)
  - [6.3 重要参数](#section-6-3)
  - [6.4 输出尺度](#section-6-4)
  - [6.5 示例配置与测试矩阵](#section-6-5)
  - [6.6 StereoBM/SBM 的 FPGA 算子、输入输出与尺寸汇总](#section-6-6)
- [7. SemiGlobalBM 与 StereoBM 对比、效果、资源和选型](#section-7)
  - [7.1 算法机制与输出差异](#section-7-1)
  - [7.2 典型效果差异](#section-7-2)
  - [7.3 资源和性能需求](#section-7-3)
  - [7.4 哪个更广泛使用](#section-7-4)
  - [7.5 对当前 AXU5EV/HXB 项目的建议](#section-7-5)
- [8. 双目校正与 stereopipeline](#section-8)
- [9. depth3D：视差转单通道深度](#section-9)
- [10. reprojectimageto3D：视差转 XYZ](#section-10)
- [11. AIE-ML Stereo Block Matching](#section-11)
- [12. 视差格式合同](#section-12)
- [13. Vitis SGBM 与 OpenCV StereoSGBM 的差异](#section-13)
- [14. 官方建议的正确使用流程](#section-14)
- [15. 当前 HXB/AXU5EV 工程与分析仓库的版本关系](#section-15)
- [16. 当前 HXB SGBM 实现与官方结构的对应关系](#section-16)
- [17. 已发现的源码/文档注意事项](#section-17)
- [18. 推荐阅读顺序](#section-18)
- [19. 官方资料与仓库内文档](#section-19)
- [20. 一句话总结](#section-20)

<a id="section-1"></a>

## 1. 结论先行

这个仓库中与双目深度直接相关的代码可以分成五组：

1. **PL 侧 SGBM（Semi-Global Block Matching）**：`xf::cv::SemiGlobalBM`。
2. **PL 侧 SAD_BM（StereoBM/SADBlockMatching）局部块匹配**：`xf::cv::StereoBM`。
3. **双目标定参数生成校正映射 + Remap + StereoBM 的完整视差流水线**：`stereopipeline`。
4. **视差到深度或三维点的转换**：`xf::cv::depth3D` 和 `xf::cv::reprojectimageto3D`。
5. **AIE-ML 侧 Stereo Block Matching**：面向带 AI Engine 的 Versal 平台，与 PL 侧 `xf_stereolbm.hpp` 是另一套实现。

最重要的结构结论是：

- 真正的 PL SGBM 算法实现只有一份，位于 [`vision/L1/include/imgproc/xf_sgbm.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_sgbm.hpp)。
- L1 和 L2 中的 `sgbm` 目录不是两份不同算法，而是同一算法在不同构建阶段的封装。
- 当前版本的 L3 中**没有** `sgbm` 目录；L3 的 [`stereopipeline`](Vitis_Libraries/vision/L3/examples/stereopipeline) 调用的是 `xf::cv::StereoBM`，不是 `xf::cv::SemiGlobalBM`。
- SGBM 和 StereoBM 的直接输出都是**视差**，不是物理深度。深度必须结合焦距、双目基线或 `Q` 矩阵另行计算。
- 当前仓库的 SGBM 输出是 `XF_8UC1` 整数视差；StereoBM 输出是 `XF_16UC1`，带 4 位小数的定点视差语义通常与 OpenCV 的 `×16` 形式对应。两者不能在不处理尺度的情况下混用。

<a id="section-2"></a>

## 2. 双目深度的端到端数据流

仓库提供的是若干可组合模块，而不是一套适合所有板卡、所有相机的固定应用。完整双目深度链路应理解为：

```text
左/右相机原始图像
        │
        ├── 双目标定（通常在离线 CPU/OpenCV 中完成）
        │     ├── 左右相机内参
        │     ├── 畸变参数
        │     ├── 双目外参
        │     ├── 校正矩阵
        │     └── Q 矩阵或焦距、基线
        │
        ▼
InitUndistortRectifyMapInverse
        │
        ▼
Remap：左右图去畸变并极线校正
        │
        ▼
视差估计
        ├── SemiGlobalBM：Census + Hamming + 多方向 SGM 聚合
        ├── StereoBM：Sobel 预处理 + SAD 局部块匹配
        └── AIE-ML Stereo Block Matching
        │
        ▼
视差图
        │
        ├── depth3D：Z = f·B/d
        └── reprojectimageto3D：Q·[x, y, d, 1]ᵀ → [X, Y, Z, W]ᵀ
        │
        ▼
深度图 / XYZ 图 / 点云
```

这里有三个必须满足的输入条件：

- 左右图像必须来自同步或足够接近同步的双目采集。
- 送入 SGBM/StereoBM 前应完成极线校正，使同一空间点尽量落在左右图的同一行。
- 输入应是尺寸一致的 8-bit 单通道灰度图；SGBM 和 PL StereoBM 都只支持 `XF_NPPC1`。

<a id="section-3"></a>

## 3. L1、L2、L3 为什么都出现相似代码

仓库的 `vision/README.md` 明确规定：单元级硬件 kernel 放在 `L1/include`，L1/L2/L3 的示例和测试通过不同方式复用这些 kernel。

| 层级 | 定位 | 双目代码中的作用 | 常见产物 |
| --- | --- | --- | --- |
| L1 | HLS 函数与模块级验证 | 包含真正算法头文件、HLS 顶层、C testbench、C 仿真和综合脚本 | RTL、Vivado IP、综合/协仿报告，也可导出 `.xo` |
| L2 | 可由 Host 调用的设备 kernel | 将 L1 算法包装成 `extern "C"` kernel，提供 OpenCL/XRT Host 和 `v++` 构建 | `.xo`、`.xclbin`、Host 程序、嵌入式 package |
| L3 | 多 kernel 应用流水线 | 将校正映射、Remap 和视差估计串成更完整应用 | 多阶段 FPGA 应用和板卡运行包 |
| `tests` | 参数化回归测试 | 复用 `examples` 源码，用不同参数组合自动构建和验证 | CSim/CSynth/CoSim/HW Emu/HW 报告 |

关系可简化为：

```text
L1/include 中唯一的算法实现
          │
          ├── L1/examples：HLS 顶层与算法级验证
          │
          ├── L1/tests：不同模板参数的 HLS 回归配置
          │
          ├── L2/examples：Vitis kernel + OpenCL/XRT Host
          │
          ├── L2/tests：设备级回归配置
          │
          └── L3/examples：多个 L1 primitive 组成应用流水线
```

因此，看到 L1、L2 中都存在 `xf_sgbm_accel.cpp`，不代表维护了两套 SGBM 数学实现。两个 wrapper 最终都包含并调用 L1 的 `imgproc/xf_sgbm.hpp`。

<a id="section-4"></a>

## 4. 仓库内双目深度相关文件总览

### 4.1 核心算法和支撑 primitive

| 文件 | 主要功能 | 是否直接生成视差/深度 |
| --- | --- | --- |
| [`L1/include/imgproc/xf_sgbm.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_sgbm.hpp) | PL SGBM：Census、Hamming cost、SGM 路径聚合、WTA | 生成整数视差 |
| [`L1/include/imgproc/xf_stereolbm.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_stereolbm.hpp) | PL StereoBM：Sobel 预处理、SAD、纹理/唯一性检查、亚像素插值 | 生成 16-bit 定点视差 |
| [`L1/include/imgproc/xf_stereo_pipeline.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_stereo_pipeline.hpp) | 根据内参、畸变系数和逆校正矩阵生成 `mapx/mapy` | 只生成校正映射 |
| [`L1/include/imgproc/xf_remap.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_remap.hpp) | 根据 `mapx/mapy` 对图像进行重映射 | 支撑双目校正，不生成视差 |
| [`L1/include/imgproc/xf_3ddepth.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_3ddepth.hpp) | 按 `Z=f·B/d` 将单通道视差转换为单通道 float 深度 | 生成深度 |
| [`L1/include/imgproc/xf_reproject3D.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_reproject3D.hpp) | 使用 4×4 `Q` 矩阵将视差重投影为 `(X,Y,Z)` | 生成三通道 XYZ |
| [`L1/include/aie-ml/imgproc/xf_sbm_impl.hpp`](Vitis_Libraries/vision/L1/include/aie-ml/imgproc/xf_sbm_impl.hpp) | AIE-ML 向量化 Stereo Block Matching 实现 | 生成 AIE 侧定点视差 |

### 4.2 L1 示例与测试

| 目录 | 说明 |
| --- | --- |
| [`L1/examples/sgbm`](Vitis_Libraries/vision/L1/examples/sgbm) | SGBM HLS wrapper、软件参考、CSim/CSynth/CoSim 入口 |
| [`L1/tests/sgbm`](Vitis_Libraries/vision/L1/tests/sgbm) | SGBM 的 `NPPC1 + 8UC1 → 8UC1` 测试配置 |
| [`L1/examples/stereolbm`](Vitis_Libraries/vision/L1/examples/stereolbm) | PL StereoBM wrapper 和参考验证 |
| [`L1/tests/stereolbm`](Vitis_Libraries/vision/L1/tests/stereolbm) | 不同 SAD 窗口、视差数、并行度及 BRAM/URAM 配置矩阵 |
| [`L1/examples/stereopipeline`](Vitis_Libraries/vision/L1/examples/stereopipeline) | HLS 级“校正映射 + Remap + StereoBM”流水线 |
| [`L1/tests/stereopipeline`](Vitis_Libraries/vision/L1/tests/stereopipeline) | 流水线的 BRAM/URAM 两类 HLS 配置 |

### 4.3 L2 示例与测试

| 目录 | 说明 |
| --- | --- |
| [`L2/examples/sgbm`](Vitis_Libraries/vision/L2/examples/sgbm) | SGBM Vitis kernel、OpenCL/XRT Host、`.xclbin` 构建 |
| [`L2/tests/sgbm`](Vitis_Libraries/vision/L2/tests/sgbm) | SGBM L2 回归配置 |
| [`L2/examples/stereolbm`](Vitis_Libraries/vision/L2/examples/stereolbm) | StereoBM Vitis kernel 和 Host |
| [`L2/tests/stereolbm`](Vitis_Libraries/vision/L2/tests/stereolbm) | StereoBM 不同参数和存储配置的设备级测试 |
| [`L2/examples/3D_depth_float`](Vitis_Libraries/vision/L2/examples/3D_depth_float) | `depth3D` Vitis kernel：视差转单通道 float 深度 |
| [`L2/tests/3D_depth_float`](Vitis_Libraries/vision/L2/tests/3D_depth_float) | `depth3D` 回归配置 |
| [`L2/examples/3D_point`](Vitis_Libraries/vision/L2/examples/3D_point) | `reprojectimageto3D` Vitis kernel：视差转 XYZ |
| [`L2/tests/3D_point`](Vitis_Libraries/vision/L2/tests/3D_point) | `reprojectimageto3D` 回归配置 |
| [`L2/tests/aie-ml/stereo_block_matching`](Vitis_Libraries/vision/L2/tests/aie-ml/stereo_block_matching) | AIE-ML StereoBM 的 GMIO 与 AIE simulation 工程 |

### 4.4 L3 示例与测试

| 目录 | 说明 |
| --- | --- |
| [`L3/examples/stereopipeline`](Vitis_Libraries/vision/L3/examples/stereopipeline) | XRT 设备级“左右校正 + StereoBM”流水线 |
| [`L3/tests/stereopipeline`](Vitis_Libraries/vision/L3/tests/stereopipeline) | L3 流水线回归配置 |

### 4.5 不应误判为双目几何代码的目录

- `3dlut` 是三维颜色查找表，不是三维点云或双目深度。
- `convertbitdepth` 是通用位深转换；它可以用于视差格式衔接，但不是视差算法。
- `mask_gen_tracking` 目录中的 `pred_depth_*.txt` 是其他视觉任务的预测深度数据，不属于双目匹配实现。

### 4.6 一个 example/test 目录中各类文件的职责

SGBM、StereoBM、3D depth 和 3D point 目录大体遵循同一种组织方式：

| 文件/目录 | 职责 |
| --- | --- |
| `config/xf_config_params.h` | 编译期图像尺寸、数据类型、视差范围、并行度等配置 |
| `xf_*_accel_config.h` | accelerator 编译入口，通常再包含 `xf_config_params.h` |
| `xf_*_accel.cpp` | 可综合的 HLS/Vitis 顶层 wrapper |
| `xf_*_tb_config.h` | testbench 编译入口 |
| `xf_*_tb.cpp` | 读取数据、软件参考、Host 调用和结果比较 |
| `run_hls.tcl` / `hls_config.tmpl` | L1 HLS CSim、CSynth、CoSim 与实现配置 |
| `Makefile` | 统一调度 HLS 或 `v++`/Host/package 构建 |
| `description.json` | 自动化测试系统读取的 case、平台、kernel 和目标元数据 |
| `utils.mk` | L2/L3 共用的平台识别、Host 架构和打包规则 |
| `xrt.ini` | XRT 仿真、日志或 profile 配置 |
| `tests/.../xf_config_params.h` | 某一具体回归参数组合；通常仍复用 `examples` 下的 accel/tb 源码 |
| `*_write_ini.tcl` | 测试自动化生成或写出 HLS 配置的辅助脚本 |

`L1/meta/api.json` 和 `L2/meta/api.json` 是 API/测试系统使用的元数据索引，不是算法的第二份实现。实际追踪算法时应回到 `L1/include`。

<a id="section-5"></a>

## 5. SemiGlobalBM：算法原理与代码实现

<a id="section-5-1"></a>

### 5.1 API 与模板参数

SGBM 顶层 API 位于 [`xf_sgbm.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_sgbm.hpp)：

```cpp
template <
    int BORDER_TYPE,
    int WINDOW_SIZE,
    int NDISP,
    int PU,
    int R,
    int SRC_T,
    int DST_T,
    int ROWS,
    int COLS,
    int NPC,
    int XFCVDEPTH_IN_L,
    int XFCVDEPTH_IN_R,
    int XFCVDEPTH_OUT>
void SemiGlobalBM(
    xf::cv::Mat<...>& left,
    xf::cv::Mat<...>& right,
    xf::cv::Mat<...>& disparity,
    uint8_t p1,
    uint8_t p2);
```

参数分为两类：

- **编译期参数**：窗口、视差范围、并行单元、聚合方向、图像上限、数据类型和像素并行度；改变它们必须重新综合和生成 bitstream。
- **运行时参数**：`p1`、`p2` 以及 wrapper 传入的实际 `rows/cols`；在生成的 IP 中通常由 AXI-Lite 配置。

<a id="section-5-2"></a>

### 5.2 算法调用链

`SemiGlobalBM` 内部用 `#pragma HLS DATAFLOW` 串起以下阶段：

```text
xf::cv::Mat 左/右图
        │
        ▼
xFCensusTransformKernel（左右各一次）
        │
        ▼
xFSGBMcomputecost
        │
        ▼
xFSGBMoptimization
        │
        ▼
xfSGBMcomputedisparity
        │
        ▼
XF_8UC1 视差图
```

源码入口关系：

```cpp
xFCensusTransformKernel(...);
xFSGBMcomputecost<NDISP, PU, ROWS, COLS>(...);
xFSGBMoptimization<NDISP, PU, R, ROWS, COLS>(..., p1, p2);
xfSGBMcomputedisparity<NDISP, PU, ROWS, COLS>(...);
```

<a id="section-5-3"></a>

### 5.3 5×5 Census Transform

`xFComputeTransform5x5` 以窗口中心像素为比较基准，对周围 24 个像素逐一比较：

```cpp
bit = (neighbor < center) ? 1 : 0;
```

最终得到 24-bit Census 描述子。它对单调光照变化通常比直接灰度差更稳健，但并不等价于 OpenCV `StereoSGBM` 默认的 Birchfield-Tomasi 代价。

边界只支持 `XF_BORDER_CONSTANT`，窗口固定为 5×5。

<a id="section-5-4"></a>

### 5.4 Hamming 匹配代价

`xFSGBMcomputecost` 为每个左图像素维护右图 Census 移位缓冲：

```cpp
xor_val = left_census ^ right_census_at_disparity;
cost = popcount(xor_val);
```

候选视差总数是 `NDISP`；每周期/每轮并行计算 `PU` 个视差，因此每像素需要大约 `NDISP/PU` 组计算。`PU` 越大，吞吐更高，但 XOR、popcount、聚合路径和存储端口资源也显著增加。

<a id="section-5-5"></a>

### 5.5 SGM 路径递推与聚合

`xFSGBMoptimization` 实现的核心递推可概括为：

```text
Lr(p,d) = C(p,d) + min(
    Lr(p-r,d),
    Lr(p-r,d-1) + P1,
    Lr(p-r,d+1) + P1,
    min_k Lr(p-r,k) + P2
) - min_k Lr(p-r,k)
```

- `P1` 惩罚相邻像素视差变化 1 的情况。
- `P2` 惩罚更大的视差跳变。
- `P1 < P2`。
- 这个实现最多聚合 4 个方向，是对完整 8/16 方向 SGM 的硬件友好近似。

主要存储是：

```cpp
uint8_t Lr[R - 1][NDISP][COLS];
```

因此 BRAM 消耗随 `R × NDISP × COLS` 增长。对于较宽图像、较大视差范围和四方向模式，`xFSGBMoptimization` 往往同时成为资源和时序热点。

<a id="section-5-6"></a>

### 5.6 Winner-Takes-All 输出

`xfSGBMcomputedisparity` 在全部候选视差中选择聚合代价最小者：

```cpp
min_disp = group_index * PU + local_min_index;
```

输出是 `XF_8UC1` 整数视差：

- `NDISP=64` 时，典型数值范围是 `0..63`。
- 不输出 OpenCV StereoBM/StereoSGBM 的 `×16` 定点值。
- 没有亚像素拟合。
- 核心 API 中没有左右一致性检查、speckle filter 和 OpenCV 风格的完整后处理参数。
- 值 `0` 同时可能代表零视差、边界/无效区域或 WTA 结果，因此下游应结合有效范围和业务规则处理。

<a id="section-5-7"></a>

### 5.7 SGBM 参数约束

当前源码中的断言约束如下：

| 参数 | 约束 |
| --- | --- |
| `SRC_T` | 必须是 `XF_8UC1` |
| `DST_T` | 必须是 `XF_8UC1` |
| `NPC` | 必须是 `XF_NPPC1` |
| `WINDOW_SIZE` | 只能是 5 |
| `NDISP` | `2 <= NDISP <= 256` |
| `PU` | `NDISP >= PU` 且 `NDISP % PU == 0` |
| `R` | 只能是 2、3 或 4 |
| `P1/P2` | `P1 < P2` |
| `P2` | 最大 100 |
| 图像尺寸 | 实际 `rows/cols` 不得超过编译期 `ROWS/COLS` |

<a id="section-5-8"></a>

### 5.8 默认示例配置

| 配置 | L1 SGBM 示例 | L2 SGBM 示例 |
| --- | ---: | ---: |
| 最大尺寸 | 1280×720 | 1920×1080 |
| Census 窗口 | 5×5 | 5×5 |
| `NDISP` | 64 | 64 |
| `PU` | 32 | 32 |
| 聚合方向 | 4 | 4 |
| 输入 | `XF_8UC1` | `XF_8UC1` |
| 输出 | `XF_8UC1` | `XF_8UC1` |
| 默认 P1/P2 | 20/40 | 20/40 |

L1 配置见 [`L1/examples/sgbm/config/xf_config_params.h`](Vitis_Libraries/vision/L1/examples/sgbm/config/xf_config_params.h)，L2 配置见 [`L2/examples/sgbm/config/xf_config_params.h`](Vitis_Libraries/vision/L2/examples/sgbm/config/xf_config_params.h)。

<a id="section-5-9"></a>

### 5.9 官方资源与性能参考

AMD 2026.1 API 文档为一组 1920×1080、64 视差、PU32、200 MHz 配置给出的参考估算是：

| 项目 | 官方参考值 |
| --- | ---: |
| BRAM_18K | 205 |
| DSP48 | 141 |
| FF | 11856 |
| LUT | 19102 |
| 估算 latency | 42 ms |

这张表只表示官方选定器件、工具和综合条件下的参考点。资源映射和 Fmax 会随目标 part、Vitis 版本、宽度、pragma 解释、接口和 Vivado 实现条件变化，不能拿它替代 ZU5EV 自己的 CSynth/implementation 报告。尤其是 SGBM 的 `Lr[R-1][NDISP][COLS]` 会让 BRAM 对图像宽度、视差数和方向数十分敏感。

<a id="section-5-10"></a>

### 5.10 SemiGlobalBM 的 L1 HLS 封装与验证路径

#### 5.10.1 HLS 顶层

[`L1/examples/sgbm/xf_sgbm_accel.cpp`](Vitis_Libraries/vision/L1/examples/sgbm/xf_sgbm_accel.cpp) 的 `semiglobalbm_accel` 负责：

```text
m_axi 左图 ─┐
            ├─ Array2xfMat ─ SemiGlobalBM ─ xfMat2Array ─ m_axi 输出
m_axi 右图 ─┘

s_axilite：P1、P2、rows、cols、ap_ctrl
```

三个图像 buffer 使用独立的 `gmem0/gmem1/gmem2` bundle，有利于在系统集成时映射到独立或可并行的 AXI/DDR 通路。

#### 5.10.2 Testbench

[`L1/examples/sgbm/xf_sgbm_tb.cpp`](Vitis_Libraries/vision/L1/examples/sgbm/xf_sgbm_tb.cpp) 完成：

1. 以灰度模式读取左右图。
2. 调用 `semiglobalbm_accel`。
3. 使用仓库自带的软件 SGM 参考实现计算对照视差。
4. 输出 `hls_out.png`、`disp_map.png` 和 `diff.png`。
5. 要求硬件模型与参考实现逐像素一致。

这个参考不是 `cv::StereoSGBM`，而是为了复现 Vitis Vision 的 Census/Hamming/四方向算法语义编写的 C++ 参考代码。

#### 5.10.3 L1 构建入口

进入：

```bash
cd vision/L1/tests/sgbm/sgbm_NPPC1_8UC1_8UC1
```

典型命令：

```bash
make run TARGET=csim   XPART=<FPGA part>
make run TARGET=csynth XPART=<FPGA part>
make run TARGET=cosim  XPART=<FPGA part>
make run TARGET=vivado_syn  XPART=<FPGA part>
make run TARGET=vivado_impl XPART=<FPGA part>
```

也可以在对应 case 中使用：

```bash
vitis-run --tcl --mode hls run_hls.tcl
```

`csim` 只验证 C/C++ 功能；只有 `csynth` 和后续 Vivado 实现才能判断资源、时钟和目标器件是否放得下。

<a id="section-5-11"></a>

### 5.11 SemiGlobalBM 的 L2 kernel 与 Host 路径

#### 5.11.1 Kernel wrapper

[`L2/examples/sgbm/xf_sgbm_accel.cpp`](Vitis_Libraries/vision/L2/examples/sgbm/xf_sgbm_accel.cpp) 与 L1 wrapper 的算法部分基本相同，主要差异是：

- 顶层使用 `extern "C"`。
- kernel 名为 `sgbm_accel`。
- 由 `v++ -c` 编译为 `.xo`，再由 `v++ -l` 链接为 `krnl_sgbm.xclbin`。

#### 5.11.2 OpenCL/XRT Host

[`L2/examples/sgbm/xf_sgbm_tb.cpp`](Vitis_Libraries/vision/L2/examples/sgbm/xf_sgbm_tb.cpp) 的主要流程是：

```text
读取两幅灰度图
  → 找到 Xilinx/AMD 设备
  → 加载 krnl_sgbm.xclbin
  → 创建 sgbm_accel kernel
  → 创建左图、右图、输出 OpenCL buffer
  → 设置 P1、P2、height、width
  → 写入输入 buffer
  → enqueueTask
  → 读回输出视差
  → 与软件参考比较
```

这个 Host 适合说明调用合同，但真实连续视频应用通常还需要：

- 持久化复用 buffer，避免每帧重复分配。
- 处理 cache/coherency 或使用合适的 XRT/CMA/驱动路径。
- 与相机采集、校正和下游深度换算做流水化。
- 统计端到端延迟，而不只统计 kernel event 延迟。

#### 5.11.3 L2 构建

PCIe 平台通常使用：

```bash
make all TARGET=hw_emu PLATFORM=<platform.xpfm>
make run TARGET=hw_emu PLATFORM=<platform.xpfm>

make all TARGET=hw PLATFORM=<platform.xpfm>
make run TARGET=hw PLATFORM=<platform.xpfm>
```

嵌入式平台还需要匹配的 `SYSROOT`、rootfs、kernel image 和 `.xpfm`，然后生成 SD card/package。

当前 `v2026.1_re` 的 SGBM L2 示例 allowlist 是 `vck190` 和 `u200`，并非面向 AXU5EV 的开箱即用工程。自定义 ZynqMP 板卡没有匹配平台时，通常应采用 L1 导出 HLS IP，再在 Vivado Block Design 中连接 PS/DDR/AXI 的路线。

<a id="section-5-12"></a>

### 5.12 SemiGlobalBM 的 FPGA 算子、输入输出与尺寸汇总

#### 5.12.1 算子层级说明

本节把源码中的 SGBM 相关函数按 HLS 使用层级分成三类：

1. **系统顶层算子**：`semiglobalbm_accel`/`sgbm_accel`，带 `m_axi` 和 `s_axilite` 接口，可以导出为 IP 或 Vitis kernel。
2. **库级算法算子**：`xf::cv::SemiGlobalBM`，输入输出是 `xf::cv::Mat`，是用户在自定义 HLS pipeline 中通常直接调用的 API。
3. **内部计算算子**：Census、cost、optimization、WTA 等函数。它们由 `SemiGlobalBM` 调用，可能被 HLS 内联、展开或实现为 dataflow process，不应当被误认为独立 `.xclbin` kernel。

#### 5.12.2 按算法流程排列的 SGBM 主算子清单

下表严格按“DDR 左右图 → 匹配代价 → 路径聚合 → 视差 → DDR”的数据流向自上而下排列。其中 `semiglobalbm_accel`/`sgbm_accel` 是整条流水线的系统边界，`SemiGlobalBM` 是包含步骤 3～9 的库级容器；它们不是在像素上额外执行一次的算法处理。

| 顺序 | 流程层级 | 主算子/处理阶段 | 输入 | 输出 | 主要参数、尺寸与硬件作用 |
| ---: | --- | --- | --- | --- | --- |
| 0 | 系统顶层 | `semiglobalbm_accel`（L1）/`sgbm_accel`（L2） | 左/右 `ap_uint<INPUT_PTR_WIDTH>*`；P1/P2；`rows/cols` | `ap_uint<OUTPUT_PTR_WIDTH>* img_out` | 两者是替代的顶层入口，而非两个先后算子；L2 版用 `extern "C"` 导出 Vitis kernel |
| 1 | DDR 读入 | `Array2xfMat` ×2 | 两路 AXI memory word | 左/右 `XF_8UC1` Mat | 每路 `H×W` 个 8-bit 像素；`H=rows`、`W=cols`；默认 AXI pointer 为 32 bit |
| 2 | 库级入口 | `xf::cv::SemiGlobalBM` | 左/右 `XF_8UC1` Mat；P1/P2 | `XF_8UC1` disparity Mat | 封装下面步骤 3～9；`WINDOW_SIZE=5`、`D=NDISP`、`P=PU`、`R`、`NPC=1` |
| 3 | Mat → stream | `SemiGlobalBM` 内联读取循环 | 左/右 Mat | 左/右 8-bit 原图 stream | 左右每路各 `H×W` 项，内部 `DATAFLOW` 的输入进程 |
| 4 | 特征变换 | **左/右 Census 5×5 ×2** | 左/右 8-bit 原图 stream | 左/右 32-bit Census 容器 stream，低 24 bit 有效 | 两幅图各调用一条 Census 链，可在 dataflow 中并行推进；每路使用 `5×COLS` 行缓存 |
| 5 | 位宽适配 | `SemiGlobalBM` 内联 32→24 bit 截取循环 | 左/右 32-bit Census 容器 stream | 左/右 `ap_uint<24>` stream | 每路 `H×W` 项；只保留 5×5 Census 的 24 个比较位 |
| 6 | 匹配代价 | `xFSGBMcomputecost` | 左/右 24-bit Census stream | `P` 路 8-bit `_cost[P]` stream | 通过 XOR+popcount 产生 Hamming cost；总量 `H×W×D`，每路 `H×W×(D/P)` 项 |
| 7 | 半全局路径聚合 | `xFSGBMoptimization` | `P` 路 8-bit cost stream；P1/P2 | `P` 路 16-bit `_agg_cost[P]` stream | 每像素聚合 `R` 个方向；状态主要随 `R×D×COLS` 增长；核心 II 目标通常为 2 |
| 8 | WTA 视差选择 | `xfSGBMcomputedisparity` | `P` 路 16-bit aggregate-cost stream | 单路 8-bit disparity stream | 每像素遍历 `D/P` 组、每组 `P` 个代价，输出 1 个整数视差 `0..D-1` |
| 9 | stream → Mat | `SemiGlobalBM` 内联写回循环 | 8-bit disparity stream | `XF_8UC1` disparity Mat | 写入 `H×W` 个 8-bit 视差，结束库内 dataflow |
| 10 | DDR 写回 | `xfMat2Array` | `XF_8UC1` disparity Mat | AXI memory word 数组 | 写回 `H×W` byte；受 `OUTPUT_PTR_WIDTH`和实际 `rows/cols` 约束 |

##### Census 为什么原来有四行：它们是四层嵌套调用，不是四个串行的整帧算子

左图和右图各有一条相同的 Census 处理链。单路内部的调用层级是：

```text
xFCensusTransformKernel                         封装/参数适配层，SGBM 传入 5×5 + constant border
└── xFCensus5x5                              整帧调度层，管理 5×COLS 行缓存和 5×5 窗口
    ├── xFProcessCensusTransform5x5           逐行处理层，滑动窗口并产生该行主体输出
    │   └── xFComputeTransform5x5         像素窗口基元，25 像素→24-bit 描述子
    └── xFComputeTransform5x5                 整帧层还直接调用它生成每行最右两个边界输出
```

| 嵌套层级 | 函数 | 单次调用的输入/输出粒度 | 与上下层的关系 |
| ---: | --- | --- | --- |
| 1 | `xFCensusTransformKernel` | 一幅图的 stream → 一幅 Census stream | 最外层 wrapper，实际调用 `xFCensus5x5` |
| 2 | `xFCensus5x5` | `H×W` 个 8-bit 像素 → `H×W` 个 32-bit 容器 | 整帧/行缓存层，循环调用逐行处理函数，并单独处理右边界 |
| 3 | `xFProcessCensusTransform5x5` | 一行的 stream 和行缓存 → 该行主体 Census 输出 | 维护 `src_buf[5][5]`，在列循环中反复调用窗口基元 |
| 4 | `xFComputeTransform5x5` | 25 个窗口像素 → 1 个 24-bit 描述子 | 最内层计算基元；以中心像素与其余 24 点比较 |

因此，主算子表中只保留一行“Census 5×5”。这四个函数表示同一阶段的软件/综合层次，不能用“前一个函数产生整帧中间图，再交给后一个函数”来理解。另外，`xFMinSAD<SIZE>::find` 也不是一个独立整帧阶段，而是被 `xFSGBMoptimization` 和 `xfSGBMcomputedisparity` 内部反复调用的树形最小值归约 helper。

#### 5.12.3 SGBM 各数据流的数量与位宽

令实际图像尺寸为 `H=rows`、`W=cols`，总视差为 `D=NDISP`，视差并行单元为 `P=PU`，方向数为 `R`：

| 数据通道 | 单项位宽 | 总项数 | 每个并行 stream 的项数 | 说明 |
| --- | ---: | ---: | ---: | --- |
| 左原图 stream | 8 bit | `H×W` | 不适用 | `XF_8UC1/NPPC1` |
| 右原图 stream | 8 bit | `H×W` | 不适用 | `XF_8UC1/NPPC1` |
| 左 Census stream | 32-bit 容器，低 24 bit 有效 | `H×W` | 不适用 | 5×5 窗口除中心外共 24 个比较位 |
| 右 Census stream | 32-bit 容器，低 24 bit 有效 | `H×W` | 不适用 | 随后显式截取为 `ap_uint<24>` |
| `_cost[P]` | 8 bit | `H×W×D` | `H×W×(D/P)` | Hamming distance 范围理论上为 0..24 |
| `_agg_cost[P]` | 16 bit | `H×W×D` | `H×W×(D/P)` | 最多累加 4 路路径代价，16 bit 足够承载 |
| `_dst` | 8 bit | `H×W` | 不适用 | 每像素一个整数视差，范围通常 `0..D-1` |

当默认 `D=64、P=32` 时，每个像素在 cost/aggregate 两级各需要两组并行处理：每组 32 个候选视差。

#### 5.12.4 SGBM 主要片上数组尺寸

| 所在算子 | 内部数组 | 元素类型 | 元素数量 | 主要映射/作用 |
| --- | --- | --- | ---: | --- |
| `xFCensus5x5` | `buf[5][COLS]` | 源像素/stream word，SGBM 中为 8 bit | `5×COLS`，左右图各一份 | `RAM_S2P BRAM`，保存 5 行形成 Census 窗口 |
| `xFCensus5x5` | `src_buf[5][5]` | 源像素 | 25 | 完全 partition，形成滑动窗口寄存器 |
| `xFSGBMcomputecost` | `r_buff[NDISP]` | `ap_uint<24>` | `NDISP` | 完全 partition，保存右图当前行的候选视差历史 |
| `xFSGBMoptimization` | `Lr[R-1][NDISP][COLS]` | `uint8_t` | `(R-1)×NDISP×COLS` | 双口 BRAM，SGBM 最大的路径状态缓存 |
| `xFSGBMoptimization` | `Lr_min[R-1][COLS]` | `uint8_t` | `(R-1)×COLS` | 保存各方向前驱位置的最小路径代价 |
| `xFSGBMoptimization` | `Lr_r0[NDISP]`、`Lr_r1[NDISP]` | `uint8_t` | 各 `NDISP` | 当前扫描方向/相邻方向的视差状态 |
| `xFSGBMoptimization` | `Lr_r1_tmp[PU]` | `uint8_t` | `PU` | 当前并行组临时结果 |
| `xFSGBMoptimization` | `tmp_store_Lr[R][PU+2]` | `uint8_t` | `R×(PU+2)` | 同时提供 `d-1/d/d+1` 三类路径候选 |
| `xFSGBMoptimization` | `store_lr_for_min[R][PU]` | `uint8_t` | `R×PU` | 每方向、每并行视差的新路径值 |
| `xfSGBMcomputedisparity` | `tmp[PU]` | `ap_uint<16>` | `PU` | 每组聚合代价的并行最小值归约 |

以 L2 默认 `COLS=1920、D=64、R=4` 为例，仅 `Lr` 的逻辑容量就是：

```text
(4 - 1) × 64 × 1920 × 8 bit = 2,949,120 bit ≈ 360 KiB
```

这还没有计入双 Census 行缓存、`Lr_min`、数组分区造成的存储碎片和其他控制/算术资源，所以 SGBM 对 BRAM 很敏感。

#### 5.12.5 SGBM 顶层 AXI buffer 尺寸

通用公式：

```text
输入字节数/路 = HEIGHT × WIDTH × 1 byte
输出字节数    = HEIGHT × WIDTH × 1 byte

输入 AXI word 数/路 = 输入字节数 ÷ (INPUT_PTR_WIDTH/8)
输出 AXI word 数    = 输出字节数 ÷ (OUTPUT_PTR_WIDTH/8)
```

默认 pointer width 都是 32 bit：

| 示例 | 最大图像 | 每路输入 | 输出 | 每路输入/输出的 32-bit word 数 |
| --- | ---: | ---: | ---: | ---: |
| L1 SGBM | 1280×720 | 921,600 byte | 921,600 byte | 230,400 |
| L2 SGBM | 1920×1080 | 2,073,600 byte | 2,073,600 byte | 518,400 |

AXI-Lite 标量参数为：

| 参数 | C++ 类型 | 有效约束/含义 |
| --- | --- | --- |
| `penalty_small` | `unsigned char` | P1，较小视差变化惩罚 |
| `penalty_large` | `unsigned char` | P2，必须 `P1<P2<=100` |
| `rows` | `int` | 实际高度，必须 `<=HEIGHT` |
| `cols` | `int` | 实际宽度，必须 `<=WIDTH` |
| `return` | HLS control | `ap_start/ap_done/ap_idle/ap_ready` 控制 |

需要注意：HLS `depth=` pragma 和模板 `HEIGHT/WIDTH`描述的是最大硬件容量；运行时只应传入实际 `rows×cols` 有效数据。Host 分配、cache flush/invalidate 和 DMA 字节数也应遵循实际有效尺寸与硬件地址合同。

<a id="section-6"></a>

## 6. StereoBM：算法原理与代码实现

> 命名说明：本文把用户所说的 **SBM** 按 Stereo Block Matching 理解。PL HLS 公开 API 名为 `xf::cv::StereoBM`，核心文件名为 `xf_stereolbm.hpp`；AIE-ML 实现则直接使用 `SbmRunner`/`xf_sbm` 命名。除第 11 章外，本章重点是 PL HLS 版本。

<a id="section-6-1"></a>

### 6.1 API、源码位置与硬件定位

PL 侧 `StereoBM` 的核心 API 和算法代码位于 [`L1/include/imgproc/xf_stereolbm.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_stereolbm.hpp)，顶层形式为：

```cpp
template <
    int WSIZE,
    int NDISP,
    int NDISP_UNIT,
    int SRC_T,
    int DST_T,
    int ROWS,
    int COLS,
    int NPC = XF_NPPC1,
    bool USE_URAM = false,
    ...>
void StereoBM(
    xf::cv::Mat<...>& left,
    xf::cv::Mat<...>& right,
    xf::cv::Mat<...>& disparity,
    xf::cv::xFSBMState<WSIZE, NDISP, NDISP_UNIT>& state);
```

它是面向 FPGA 流式实现的局部块匹配器：只在每个像素附近的有限窗口中计算代价，不执行跨整幅图像的多路径全局优化。算法结构简单、存储范围有限，并提供 BRAM/URAM 两种内部存储映射。

<a id="section-6-2"></a>

### 6.2 核心调用链

[`xf_stereolbm.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_stereolbm.hpp) 的主要阶段为：

```text
左右 XF_8UC1
    │
    ▼
xFStereoPreProcess
    ├── 3×3 Sobel X/Y
    ├── X 梯度裁剪并平移到 8-bit
    └── Y 梯度仅被读出以释放数据流
    │
    ▼
xFSADBlockMatching
    ├── 窗口 SAD 增量更新
    ├── 每轮并行 NDISP_UNIT 个视差
    ├── 纹理阈值过滤
    ├── uniqueness 检查
    ├── WTA 最小 SAD
    └── 邻域代价二次拟合/亚像素插值
    │
    ▼
XF_16UC1 disparity
```

<a id="section-6-3"></a>

### 6.3 重要参数

`xFSBMState` 在 wrapper 中主要由以下运行时字段控制：

- `preFilterCap`
- `uniquenessRatio`
- `textureThreshold`
- `minDisparity`

窗口 `WSIZE`、总视差 `NDISP`、并行视差单元 `NDISP_UNIT` 和是否使用 URAM 则是编译期参数。

主要约束：

- 输入必须为 `XF_8UC1`。
- 输出必须为 `XF_16UC1`。
- 只支持 `XF_NPPC1`。
- SAD 窗口必须为奇数，至少 5，示例/文档测试范围通常不超过 21。
- `NDISP > 1` 且小于图像宽度。
- `NDISP % NDISP_UNIT == 0`。
- `preFilterCap` 为 `1..63`。
- 预滤波类型必须是 Sobel 类型。

<a id="section-6-4"></a>

### 6.4 输出尺度

StereoBM 在候选视差最小值附近用相邻 SAD 代价估计亚像素偏移，最终输出保留 4 位小数的定点视差。实际使用时应把它理解为：

```text
真实像素视差 d = disparity_u16 / 16.0
```

下游调用 `depth3D` 或使用普通物理公式时，应先明确是否已经除以 16。否则深度会缩小 16 倍。

<a id="section-6-5"></a>

### 6.5 示例配置与测试矩阵

L1 默认配置：

```text
1280×720
SAD window = 11
NDISP = 32
NDISP_UNIT = 32
XF_USE_URAM = false
XF_8UC1 → XF_16UC1
```

L2 默认配置把尺寸改为 1920×1080，其他主参数相同。

`L1/tests/stereolbm` 和 `L2/tests/stereolbm` 覆盖以下典型组合：

| SAD 窗口 | 总视差 | 并行视差单元 | 存储配置 |
| ---: | ---: | ---: | --- |
| 5 | 16 | 2 | BRAM / URAM |
| 9 | 32 | 4 | BRAM / URAM |
| 11 | 32 | 32 | BRAM / URAM |
| 15 | 128 | 32 | BRAM / URAM |
| 21 | 64 | 16 | BRAM / URAM |

这些目录名就是配置合同，例如：

```text
stereolbm_NPPC1_8UC1_16UC1_15_128_32
stereolbm_NPPC1_8UC1_16UC1_URAM_15_128_32
```

<a id="section-6-6"></a>

### 6.6 StereoBM/SBM 的 FPGA 算子、输入输出与尺寸汇总

#### 6.6.1 算子层级说明

StereoBM 同样分成三层：

1. **系统顶层算子**：`stereolbm_accel`，提供 DDR `m_axi`、4 字节状态输入和 AXI-Lite 控制。
2. **库级算法算子**：`xf::cv::StereoBM`，用户在 HLS pipeline 中调用。
3. **内部算子**：Sobel/clipping、滑窗、SAD 增量更新、纹理和唯一性过滤、最小值归约、亚像素插值。这些通常被内联或成为 dataflow 内部 process。

#### 6.6.2 按算法流程排列的 StereoBM 主算子清单

下表只描述默认 **Mat 入口路径**，并严格按像素数据的先后流向排列。`xFSBMState` 是同时送入多个阶段的配置支路，stream/SPC 函数是另一种入口，都不应夹在主像素流中当作先后算子。

| 顺序 | 流程层级 | 主算子/处理阶段 | 输入 | 输出 | 主要参数、尺寸与硬件作用 |
| ---: | --- | --- | --- | --- | --- |
| 0 | 系统顶层 | `stereolbm_accel` | 左/右 `ap_uint<INPUT_PTR_WIDTH>*`；`bm_state_in[4]`；`rows/cols` | `ap_uint<OUTPUT_PTR_WIDTH>* img_out` | 整条 DDR→StereoBM→DDR 流水线的接口边界；模板确定 `HEIGHT/WIDTH`、`S=WSIZE`、`D=NDISP`、`U=NDISP_UNIT`、`USE_URAM` |
| 1 | DDR 读入 | `Array2xfMat` ×2 | 两路 AXI memory word | 左/右 `XF_8UC1` Mat | 每路 `H×W` 个 8-bit 像素，只支持 `NPPC1` |
| 2 | 库级入口 | `xf::cv::StereoBM` | 左/右 `XF_8UC1` Mat；`xFSBMState` | `XF_16UC1` disparity Mat | 封装下面步骤 3～8；逻辑输入/输出尺寸均为 `H×W` |
| 3 | 参数校验/调度 | `xFFindStereoCorrespondenceLBM` | 左/右 Mat、状态、H/W | 传给下层的左/右 Mat 和状态 | 校验类型、窗口、视差和 cap；计算 `SWEEP_FACT=ceil(D/U)`；调用 `...LBMNO` |
| 4 | dataflow 编排 | `xFFindStereoCorrespondenceLBMNO` | 左/右 Mat；`xFSBMState` | disparity Mat | 建立左/右 clipped stream 和 disparity stream，用 `DATAFLOW` 封装步骤 5～8 |
| 5 | 左/右预处理 | `xFStereoPreProcess` ×2 | 左/右 `XF_8UC1` Mat；`preFilterCap` | 左/右 8-bit clipped stream | 两路各处理 `H×W` 像素，可并行推进；单路内部是 `Sobel 3×3 → X 梯度裁剪`，Y 梯度另行消费 |
| 6 | 局部块匹配 | `xFSADBlockMatching` | 左/右 8-bit clipped stream；`xFSBMState`；H/W | 16-bit Q12.4 disparity stream | 在一个算子内完成行缓存、滑窗、增量 SAD、纹理阈值、uniqueness、WTA 和亚像素拟合；每轮并行 `U` 个视差，共 `ceil(D/U)` 轮 |
| 7 | 结果收集 | `xFFindStereoCorrespondenceLBMNO` 内联写 Mat 循环 | 16-bit Q12.4 disparity stream | `XF_16UC1` disparity Mat | 循环 `H×W` 次，II=1；每次从 `_disp_strm` 读一项并写入 Mat |
| 8 | DDR 写回 | `xfMat2Array` | `XF_16UC1` disparity Mat | AXI memory word | 写回 `H×W` 个 16-bit Q12.4 视差，真实像素视差为 `raw/16.0` |

##### StereoBM 配置支路：不占据像素流的一个顺序阶段

`xFSBMState<WSIZE,NDISP,NDISP_UNIT>` 在进入 `StereoBM` 之前构造，但它不接收、也不输出一幅图。它是主数据流的侧边配置输入：

| 配置对象 | 输入 | 输出/消费者 | 关系 |
| --- | --- | --- | --- |
| `xFSBMState<WSIZE,NDISP,NDISP_UNIT>` | 编译期 `WSIZE/NDISP/NDISP_UNIT`；wrapper 的 `bm_state_in[4]` | `StereoBM`、预处理、SAD/过滤/视差选择阶段 | 携带 `preFilterCap`、`uniquenessRatio`、`textureThreshold`、`minDisparity` 等控制量，并计算 sweep 相关状态 |

##### `xFStereoPreProcess` 的嵌套调用顺序

```text
单路 XF_8UC1 Mat
    └── xf::cv::Sobel (3×3, constant border)
        ├── Sobel-X XF_16SC1 → xFImageClip → 8-bit clipped stream → 匹配核
        │                         └── xFImageClipUtility（内部边界 helper）
        └── Sobel-Y XF_16SC1 → xFReadOutStream → 丢弃，仅为防止 dataflow 堵塞
```

`Sobel-X` 和 `Sobel-Y` 是同一次 Sobel 的两个输出支路，不是两个先后图像阶段。`xFImageClipUtility` 也只是 `xFImageClip` 内部的像素边界 helper。

##### `xFSADBlockMatching` 内部 helper 与主处理的关系

下列函数都在 `xFSADBlockMatching` 的行/列/sweep 循环内反复执行，它们共同实现主表第 6 步，不是多个各自产生整帧中间图的串行 kernel：

| 在核心循环中的作用顺序 | helper | 输入 | 输出/作用 |
| ---: | --- | --- | --- |
| 1 | `xFUpdateTextureSum` | 当前左窗口、待插入的新列、`preFilterCap` | 在窗口移位之前滚动更新 `text_sum[WSIZE]`，为 texture threshold 判断提供代价 |
| 1a | `xFabsdiff2` | 两个标量 | 上一步内部使用的标量绝对差基元 |
| 2 | `xFShiftRight`、`xFInsertLeft` | 滑窗数组和新列 | 先右移左/右匹配窗口，再把新像素列插入第 0 列 |
| 3 | `xFSADComputeInc` | 左/右窗口、局部视差 `d`、`sad_cols_d[WSIZE]` | 加新列、减旧列，输出并更新窗口 SAD，避免整窗重算 |
| 4 | `xFMinSAD<NDISP_UNIT>::find` | 当前 sweep 的 `U` 个 SAD | 树形归约得到局部最小 SAD 及其索引，参与跨 sweep 的 WTA |
| 5 | `xFSADBlockMatching` 内联判断/拟合逻辑 | 纹理值、最小和次小 SAD、相邻视差 SAD | uniqueness/纹理过滤后的 Q12.4 亚像素视差 |

##### stream/SPC 是替代入口，不是 Mat 主路径的中间阶段

| 替代路径 | 输入 | 输出 | 与主路径的关系 |
| --- | --- | --- | --- |
| `xFFindStereoCorrespondenceLBM_pipeline` → `xFFindStereoCorrespondenceLBMNO_pipeline` | 左/右 8-bit stream | 16-bit disparity pointer/stream | 用于 stream/SPC 调用，代替 Mat 版的 `xFFindStereoCorrespondenceLBM` → `...LBMNO`；内部仍调用左/右预处理和同一 `xFSADBlockMatching` |

#### 6.6.3 StereoBM 各数据流数量与位宽

令 `H=rows`、`W=cols`、窗口边长 `S=WSIZE`、总视差 `D=NDISP`、每轮并行视差 `U=NDISP_UNIT`：

| 数据通道 | 单项位宽 | 项数 | 说明 |
| --- | ---: | ---: | --- |
| 左/右原图 | 各 8 bit | 各 `H×W` | `XF_8UC1/NPPC1` |
| Sobel X | 16-bit signed | `H×W` | 内部 `XF_16SC1` Mat |
| Sobel Y | 16-bit signed | `H×W` | 算法不用数值，但必须消费以完成 dataflow |
| 左/右 clipped stream | 各 8 bit | 各 `H×W` | X 梯度经 cap 和偏移后的值 |
| 每个 sweep 的 SAD 向量 | 每候选通常用 32-bit `int` 累计 | 每像素 `U` 个 | 共执行 `ceil(D/U)` 次 sweep |
| 最终 disparity stream | 16 bit unsigned | `H×W` | Q12.4，即像素视差约为 `raw/16.0` |

核心扫描循环的逻辑迭代范围为：

```text
row iterations   = H + S - 1
column iterations= W + S - 1
sweep iterations = ceil(D / U)
```

在核心列循环目标 `II=1` 时，可用下面的数量级理解纯匹配周期数：

```text
(H + S - 1) × (W + S - 1) × ceil(D/U)
```

实际顶层 latency 还包含 Sobel、填充/排空 pipeline 和数据搬运。

#### 6.6.4 `xFSADBlockMatching` 的主要片上数组尺寸

源码实例化时通常有：

```text
BUF_SIZE   = COLS + WSIZE - 1
LWINWIDTH  = WSIZE
RWINWIDTH  = WSIZE + NDISP_UNIT - 1
SWEEP_FACT = ceil(NDISP / NDISP_UNIT)
```

主要数组如下：

| 内部数组 | 元素类型 | 元素数量 | 作用/映射 |
| --- | --- | ---: | --- |
| `left_line_buf[WSIZE][BUF_SIZE]` | 8-bit 源 stream word | `S×(COLS+S-1)` | 左图行缓存；`USE_URAM` 时映射 URAM，否则按第 1 维 partition |
| `right_line_buf[WSIZE][BUF_SIZE]` | 8-bit 源 stream word | `S×(COLS+S-1)` | 右图行缓存，映射同上 |
| `l_window[WSIZE][LWINWIDTH]` | `unsigned char` | `S×S` | 左匹配窗口，完全 partition |
| `r_window[WSIZE][RWINWIDTH]` | `unsigned char` | `S×(S+U-1)` | 同时容纳 U 个右图候选视差窗口，完全 partition |
| `l_tmp[WSIZE]`、`r_tmp[WSIZE]` | `unsigned char` | 各 `S` | 每列新读入像素 |
| `text_sum[WSIZE]` | `int` | `S` | 窗口纹理强度滚动状态 |
| `sad[NDISP_UNIT]` | `int` | `U` | 当前 sweep 内 U 个候选视差的窗口 SAD |
| `sad_cols[NDISP_UNIT][WSIZE]` | `short int` | `U×S` | 每候选视差的列 SAD 队列，用于增量加减 |
| `minsad[COLS+WSIZE-1]` | `int` | `COLS+S-1` | 多 sweep 之间保存当前全局最小 SAD |
| `mind[BUF_SIZE]` | 16-bit disparity word | `COLS+S-1` | 多 sweep 之间保存当前最小视差索引 |
| `skip[BUF_SIZE]` | `bool` | `COLS+S-1` | 唯一性/纹理过滤状态 |
| `skip_val/edge_neighbor/edge/minsad_p/minsad_n[BUF_SIZE]` | `int` | 每个数组 `COLS+S-1` | 唯一性判断和亚像素拟合需要的跨 sweep 状态；具体 BRAM/URAM 映射由 HLS 决定 |

与 SGBM 的 `R×D×COLS` 路径状态相比，StereoBM 的关键缓存主要随 `S×COLS`、`U×S` 和若干 `COLS` 状态数组增长。这就是局部 BM 通常更节省 BRAM 的根本原因。

#### 6.6.5 `xFSBMState` 参数合同

完整状态类在 [`L1/include/common/xf_structs.hpp`](Vitis_Libraries/vision/L1/include/common/xf_structs.hpp) 中定义，包含 11 个 `int` 字段：

| 字段 | 默认值/来源 | 当前 PL 算法中的作用 |
| --- | --- | --- |
| `preFilterType` | `XF_STEREO_PREFILTER_SOBEL_TYPE` | 固定要求 Sobel 预滤波 |
| `preFilterSize` | `WSIZE` | 状态描述字段；实际 Sobel 核固定 3×3，匹配窗口由模板 WSIZE 控制 |
| `preFilterCap` | 31，可运行时覆盖 | Sobel-X clipping 范围，要求 1..63 |
| `SADWindowSize` | `WSIZE` | 状态描述字段；真正硬件尺寸由模板 WSIZE 决定 |
| `minDisparity` | 0，可运行时覆盖 | 最小视差合同；当前核心输出/过滤路径对它的使用有限，不能等同 OpenCV 全功能实现 |
| `numberOfDisparities` | `NDISP` | 状态描述字段；真正硬件规模由模板 NDISP 决定 |
| `textureThreshold` | 10，可运行时覆盖 | 低纹理窗口过滤阈值 |
| `uniquenessRatio` | 15，可运行时覆盖 | 最优 SAD 与其他候选的唯一性检查百分比 |
| `ndisp_unit` | `NDISP_UNIT` | 描述每 sweep 并行候选数 |
| `sweepFactor` | `ceil(NDISP/NDISP_UNIT)` | 核心匹配需要扫描的 sweep 数 |
| `remainder` | `U×sweepFactor-D` | 最后一组无效 lane 数 |

Standalone L1/L2 `stereolbm_accel` 的外部 `bm_state_in` 只有 4 个 `unsigned char`：

```text
bm_state_in[0] = preFilterCap
bm_state_in[1] = uniquenessRatio
bm_state_in[2] = textureThreshold
bm_state_in[3] = minDisparity
```

其余字段由 `xFSBMState` 构造函数和模板参数确定。相反，L3 `stereopipeline_accel` 接收 `int* bm_state_arr` 并装载 11 个字段；但其中由模板固定的尺寸字段即使通过数组传入，也不能在不重新综合的情况下真正改变硬件窗口、总视差或并行度。

还要注意，`xFSBMState::minDisparity` 本身是 `int`，理论上可以描述负值；但 standalone wrapper 的 `bm_state_in[3]` 是 `unsigned char`，无法直接表达负 `minDisparity`。如果应用需要负视差搜索，必须重新审查并修改 wrapper、核心索引逻辑和 Host 合同，不能只向这个字节写入补码后假定行为等价于 OpenCV。

#### 6.6.6 StereoBM 顶层 AXI buffer 尺寸

通用公式：

```text
输入字节数/路 = HEIGHT × WIDTH × 1 byte
输出字节数    = HEIGHT × WIDTH × 2 byte

输入 AXI word 数/路 = 输入字节数 ÷ (INPUT_PTR_WIDTH/8)
输出 AXI word 数    = 输出字节数 ÷ (OUTPUT_PTR_WIDTH/8)
状态输入             = 4 × unsigned char
```

默认 pointer width 是 32 bit：

| 示例 | 最大图像 | 每路输入 | 16-bit 输出 | 输入 32-bit word/路 | 输出 32-bit word |
| --- | ---: | ---: | ---: | ---: | ---: |
| L1 StereoBM | 1280×720 | 921,600 byte | 1,843,200 byte | 230,400 | 460,800 |
| L2 StereoBM | 1920×1080 | 2,073,600 byte | 4,147,200 byte | 518,400 | 1,036,800 |

顶层接口 bundle 为：左图 `gmem0`、右图 `gmem1`、4-byte 状态 `gmem2`、输出 `gmem3`，`rows/cols/return` 通过 AXI-Lite control。由于输出是输入字节数的两倍，Host buffer、DDR 地址规划和 cache 同步长度不能沿用 SGBM 的 8-bit 输出尺寸。

<a id="section-7"></a>

## 7. SemiGlobalBM 与 StereoBM 对比、效果、资源和选型

<a id="section-7-1"></a>

### 7.1 算法机制与输出差异

| 对比项 | `SemiGlobalBM` | `StereoBM` |
| --- | --- | --- |
| 算法类别 | 半全局匹配 | 局部块匹配 |
| 预处理/描述子 | 固定 5×5 Census | 3×3 Sobel X 梯度和 clipping |
| 匹配代价 | Census 描述子的 Hamming distance | 匹配窗口内的 SAD |
| 代价优化 | 沿 2/3/4 个方向进行 SGM 路径递推和聚合 | 只在当前局部窗口内累计代价 |
| 平滑/过滤参数 | P1、P2 | `textureThreshold`、`uniquenessRatio`、`preFilterCap` |
| 最终选点 | 聚合代价 WTA | SAD 代价 WTA + 邻域代价亚像素拟合 |
| Vitis 输出 | `XF_8UC1` 整数视差 | `XF_16UC1` Q12.4 定点视差 |
| 亚像素精度 | 当前实现没有 | 有，4 位小数 |
| 无效值 | 通常由 0、边界和下游规则共同解释 | 纹理/唯一性过滤失败输出 0 |
| PL 主要存储压力 | 路径状态 `R×NDISP×COLS` | 窗口/行缓存、SAD 状态，可选 URAM |

<a id="section-7-2"></a>

### 7.2 典型效果差异

在输入已经正确极线校正、曝光较一致的前提下，两者通常呈现以下差异：

| 场景 | `SemiGlobalBM` 的典型表现 | `StereoBM` 的典型表现 |
| --- | --- | --- |
| 弱纹理平面 | 多路径平滑约束能维持更连续的视差 | 局部窗口缺少可靠特征，容易出现空洞和噪声 |
| 物体边缘 | 通常比纯局部匹配稳定，但 P2 过大也会把边界抹平 | 大窗口会跨越前后景，容易产生边缘膨胀/前景拖尾 |
| 重复纹理 | 路径上下文有助于减少部分歧义 | 局部窗口中可能出现多个相近 SAD 最小值 |
| 光照单调变化 | Census/Hamming 相对稳健 | 依赖梯度和 SAD，对曝光/响应差异更敏感 |
| 遮挡区域 | 仍可能错误，当前 API 没有完整左右一致性检查 | 通常出现无效值、错误块或边界伪影 |
| 细小结构 | 整数视差且平滑聚合可能损失极细结构 | 小窗口可能保留细节，但噪声更大；大窗口会抹除细节 |
| 深度精细度 | 当前 Vitis 实现只有整数像素视差 | Q12.4 亚像素输出在近似正确匹配处可提供更细深度层级 |

因此，“SGBM 效果一定更好”也不是无条件成立：它通常带来更连续、抗弱纹理更好的视差，但当前 Vitis 版本缺少亚像素输出；StereoBM 在纹理丰富、基线和分辨率合适、窗口调参良好的场景中，可能以更低资源获得足够效果，并拥有更细的 Q12.4 数值分辨率。

最终效果还高度依赖：标定/校正误差、相机同步、曝光一致性、视差搜索范围、窗口或 P1/P2、无效点过滤以及深度后处理。算法名本身不能替代同一数据集上的实测。

<a id="section-7-3"></a>

### 7.3 资源和性能需求

资源趋势上，SemiGlobalBM 通常比 StereoBM 更重，原因是它必须长期保存多个方向、全部候选视差和图像列对应的路径状态；StereoBM 主要保存局部窗口、行缓存和分组 SAD 状态。

仓库文档给出的两个参考点如下，但二者的工具版本、器件、频率和视差配置不同，**只能用于观察数量级，不能作为严格横向 benchmark**：

| 官方参考配置 | BRAM_18K | DSP | FF | LUT | 性能 |
| --- | ---: | ---: | ---: | ---: | --- |
| SemiGlobalBM：1920×1080、D64、PU32、200 MHz | 205 | 141 | 11856 | 19102 | 约 42 ms |
| StereoBM：1920×1080、D32、PU32、300 MHz，U200 benchmark | 13 | 7 | 20670 | 19380 | 约 135 FPS |

另一张较早的 StereoBM HLS 表对 HD、窗口 11、D32、PU32 给出 49 个 BRAM、20 个 DSP、34519 个 FF 和 31978 个 LUT；这也说明资源会随工具、器件、wrapper 和报告口径明显变化。选型时必须在目标 part、目标尺寸和相同约束下分别跑 CSynth/Vivado implementation。

总体资源规律是：

- SemiGlobalBM 的 BRAM 对 `COLS`、`NDISP` 和方向数 `R` 非常敏感。
- 增大 SemiGlobalBM 的 `PU` 会提高并行度，同时增加组合逻辑、DSP/加法器和存储端口压力。
- StereoBM 的资源随 SAD 窗口、`NDISP`、`NDISP_UNIT` 增长，可通过 `USE_URAM` 把部分 BRAM 压力转移到 URAM。
- StereoBM 并行度较小时可能需要多次 sweep，资源较低但 latency 增加。
- 两者的端到端速度还受 DDR、cache 同步、Host 调度和校正/后处理影响，不能只看 kernel latency。

<a id="section-7-4"></a>

### 7.4 哪个更广泛使用

需要按使用范围分别回答：

1. **在经典软件双目和 OpenCV 应用中**，`StereoSGBM`/SGM 家族通常比基础 `StereoBM` 更常作为“质量优先”的工程起点，因为它在弱纹理和连续表面上的结果通常更稳。OpenCV 同时长期提供 `StereoBM` 和 `StereoSGBM`，简单或教学场景仍大量使用 StereoBM。
2. **在资源受限 FPGA/嵌入式硬件中**，StereoBM 往往更容易落地，也更容易与校正流水线放在同一器件中；它的局部结构、较低片上存储压力和可预测数据流非常适合硬件化。
3. **在当前 Vitis Vision 仓库中**，StereoBM 的覆盖面更广：有 PL L1/L2、L1/L3 `stereopipeline`，还有 AIE-ML SBM；SemiGlobalBM 主要提供 PL L1/L2 路径。就这个仓库的集成广度而言，StereoBM 更广泛。
4. **在当前高质量双目研究和产品中**，深度学习双目网络也很常见，因此不能把“经典算法中 SGBM 更普遍”扩展成所有现代双目方案中的绝对结论。

一句话概括：

```text
质量和弱纹理连续性优先：先评估 SemiGlobalBM/SGM。
资源、功耗、确定性和易集成优先：先评估 StereoBM。
```

<a id="section-7-5"></a>

### 7.5 对当前 AXU5EV/HXB 项目的建议

当前 HXB 的 SemiGlobalBM 已经综合并上板，且 ZU5EV 资源报告显示 BRAM/LUT 较紧。因此：

- 如果目标是保持现有较好的连续视差效果，应继续使用现有 SemiGlobalBM，优化 PS/DDR/cache 和后处理，不要仅为降低少量 kernel latency 改回 StereoBM。
- 如果以后必须把校正、Remap、视差和更多视觉模块全部放入 PL，StereoBM 值得作为资源回退方案重新综合比较。
- 两者必须使用同一批校正图、相同有效 ROI 和统一的视差/深度尺度评估；至少比较有效像素率、坏点率、边缘误差、深度 RMSE、PL 资源、Fmax 和端到端 FPS。
- 不能直接拿官方不同器件、不同视差范围的表格替代 ZU5EV 的同条件综合结果。

<a id="section-8"></a>

## 8. 双目校正与 `stereopipeline`

### 8.1 `InitUndistortRectifyMapInverse`

[`xf_stereo_pipeline.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_stereo_pipeline.hpp) 根据以下参数生成目标图像到原始图像的逆映射：

- `cameraMatrix`：相机内参。
- `distCoeffs`：径向/切向畸变参数。
- `ir`：通常是逆校正旋转与新相机矩阵组合后的 3×3 矩阵。

对每个目标像素计算归一化坐标、畸变坐标和原图采样位置，并把 `mapx/mapy` 以乘 256 的定点形式写入 `XF_32SC1` map。

### 8.2 Remap

`xf::cv::remap` 使用上述 map 对左右图做双线性插值。`XF_REMAP_BUFSIZE` 是必须重点配置的行缓存/重映射缓冲参数；它过小会无法覆盖映射所需的垂直访问范围，过大则增加片上存储。

### 8.3 L1 与 L3 流水线

L1 [`examples/stereopipeline`](Vitis_Libraries/vision/L1/examples/stereopipeline) 用于 HLS 级验证；L3 [`examples/stereopipeline`](Vitis_Libraries/vision/L3/examples/stereopipeline) 增加设备级 Host、XRT/构建和打包逻辑。两者的核心数据流都是：

```text
左原图 → 生成左 map → Remap ─┐
                              ├→ StereoBM → XF_16UC1 视差
右原图 → 生成右 map → Remap ─┘
```

L1 默认配置为 1280×720；L3 默认配置为 1920×1080。两者典型匹配参数为：

```text
NDISP = 48
NDISP_UNIT = 16
SAD window = 15
Remap buffer = 128
NPPC1
```

### 8.4 为什么它不是 SGBM pipeline

流水线末端源码明确调用：

```cpp
xf::cv::StereoBM<...>(leftRemappedMat, rightRemappedMat, mat_disp, bm_state);
```

没有调用 `SemiGlobalBM`。如果需要“校正 + SGBM”，可以在概念上替换末端，但必须同步处理：

1. 输出类型从 `XF_16UC1` 改为 `XF_8UC1`。
2. 删除 `xFSBMState`，换成 SGBM 的 P1/P2。
3. 增加 SGBM 的 `NDISP/PU/R/WINDOW_SIZE` 编译参数。
4. 修改 Host buffer 字节数和视差尺度解释。
5. 重新评估 BRAM/LUT/DSP 和时序。
6. 检查 `InitUndistortRectifyMapInverse + Remap + SGBM` 在目标器件上是否能同时放下。

对于资源较紧的 ZynqMP，常见工程折中是让 PS/OpenCV 完成校正，PL 只计算 SGBM；这样可显著降低 PL 静态资源压力。

<a id="section-9"></a>

## 9. `depth3D`：视差转单通道深度

### 9.1 实际源码合同

[`xf_3ddepth.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_3ddepth.hpp) 实现：

```text
if disparity == 0:
    depth = 0
else:
    depth = focal_len * base_dis / disparity
```

实际源码断言：

- 输入：`XF_16UC1` 或 `XF_16SC1`。
- 输出：`XF_32FC1`。
- 并行度：`XF_NPPC1`。

L2 wrapper 位于 [`L2/examples/3D_depth_float/xf_3ddepth_accel.cpp`](Vitis_Libraries/vision/L2/examples/3D_depth_float/xf_3ddepth_accel.cpp)，通过 AXI-Lite 接收 `focal_len`、`base_dis`、`height` 和 `width`。

### 9.2 单位关系

公式：

```text
Z = f × B / d
```

- `f` 用像素表示。
- `d` 用像素视差表示。
- `B` 用米，输出就是米；`B` 用毫米，输出就是毫米。

### 9.3 与两种视差格式的衔接

SGBM 输出：

```text
XF_8UC1，d 已是整数像素
```

接入 `depth3D` 前只需无损扩展为 `XF_16UC1/16SC1`，不要额外乘 16。

StereoBM/OpenCV StereoBM 输出：

```text
XF_16UC1/CV_16S，通常保存 d×16
```

接入物理公式前应先得到像素视差：

```text
d_pixels = d_fixed / 16.0
```

当前 `3D_depth_float` testbench 直接把 OpenCV StereoBM 的原始 `CV_16S` 结果送给 `depth3D`，参考函数也用相同原始值，因此它可以做到内部逐值一致，但按真实焦距/基线解释时会带有 `×16` 尺度问题。生产代码必须自行统一视差单位。

### 9.4 文档与源码不一致

当前 [`3Ddepth_api.rst`](Vitis_Libraries/vision/docs/src/include/3Ddepth_api.rst) 的参数表写成了三通道输入/输出，而 C++ 源码和 L2 配置实际是：

```text
XF_16UC1 或 XF_16SC1 → XF_32FC1
```

这是当前仓库文档中的明显不一致。工程使用时应以头文件模板断言、示例配置、CSim 和综合结果为准。

<a id="section-10"></a>

## 10. `reprojectimageto3D`：视差转 XYZ

### 10.1 算法

[`xf_reproject3D.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_reproject3D.hpp) 对每个像素构造：

```text
[Xh, Yh, Zh, Wh]ᵀ = Q × [x, y, d, 1]ᵀ

X = Xh / Wh
Y = Yh / Wh
Z = Zh / Wh
```

源码使用 16 个 `short int` 表示 Q，并以整数乘加和整数除法计算，输出 `XF_16SC3`。因此它不是 OpenCV `reprojectImageTo3D` 的通用 double/float 精度版本。

### 10.2 实际源码合同

- 输入：`XF_16UC1` 或 `XF_16SC1`。
- 输出：`XF_16SC3`。
- Q：`short int[16]`，testbench 将 double Q 量化为带 5 个小数位的定点形式。
- `min_disp`：由 Host 计算并传入。
- `handle_missval=true` 时，最小视差对应点的 Z 被置为 10000。
- 视差为 0 时，源码输出 `(0,0,-1)`。

L2 wrapper 位于 [`L2/examples/3D_point/xf_reprojectimageto3d_accel.cpp`](Vitis_Libraries/vision/L2/examples/3D_point/xf_reprojectimageto3d_accel.cpp)。

### 10.3 视差尺度

官方 RST 特别提醒：如果输入是 StereoBM/StereoSGBM 生成的 16-bit `×16` 视差，使用前应先除以 16。对于本仓库 SGBM 的 8-bit 整数视差，则只需扩展为 16-bit，并确保 Q 按“像素视差”构造。

### 10.4 文档与源码不一致

[`3Dpoint_api.rst`](Vitis_Libraries/vision/docs/src/include/3Dpoint_api.rst) 的 API 片段描述了 `double* Q` 和 float 输出，但当前头文件实际接收 `short int* Q_mtrx`，输出 `XF_16SC3`。使用当前 checkout 时必须遵循源码，而不能直接照抄 RST 签名。

此外，`xf_reproject3D.hpp` 内部包含了一次自身头文件；include guard 使其不会递归展开，但这是无实际作用的冗余 include。

<a id="section-11"></a>

## 11. AIE-ML Stereo Block Matching

### 11.1 位置与适用平台

AIE-ML 实现位于：

```text
vision/L1/include/aie-ml/imgproc/xf_sbm_impl.hpp
vision/L2/tests/aie-ml/stereo_block_matching/
```

它面向带 AI Engine/AIE-ML 的 Versal 设备，并不适用于没有 AI Engine 的 ZU5EV 或 Zynq-7000。

### 11.2 工程结构

```text
stereo_block_matching/
├── README.txt
├── gmio/
│   ├── graph.h / graph.cpp      ADF graph、GMIO、shared_buffer、tile 位置
│   ├── kernels.h                AIE kernel 类声明
│   ├── xf_sobel.cc              Sobel kernel wrapper
│   ├── xf_sbm.cc                SBM kernel wrapper
│   ├── host.cpp                 OpenCV 参考、分块、GMIO 传输和结果验证
│   ├── config.h                 图像、tile、视差和核数量参数
│   └── Makefile/system.cfg
└── gmio_aiesim/
    └── 对应 AIE simulation 配置
```

`graph.h` 中先以多个 `SobelRunner` 做预处理，再用多个 `SbmRunner` 处理分块；GMIO 与 MemTile shared buffer 负责 DDR、Memory Tile 和 AIE kernel 间的数据移动。

### 11.3 AIE 约束

其 `README.txt` 给出：

- `NO_DISPARITIES` 必须是 16 的倍数。
- `16 <= NO_DISPARITIES <= 240`。
- `TILE_WINSIZE` 必须是 5 到 21 之间的奇数。
- `gmio_aiesim` 用于 AIE 仿真。

它实现的是局部 Stereo Block Matching，不是 SGM/SGBM。

<a id="section-12"></a>

## 12. 视差格式合同：最容易出错的地方

| 产生者 | 输出类型 | 数值语义 | 接 `depth3D/Q` 前的处理 |
| --- | --- | --- | --- |
| Vitis `SemiGlobalBM` | `XF_8UC1` | 整数像素视差 `0..NDISP-1` | 扩展为 16-bit，保持数值不变 |
| Vitis PL `StereoBM` | `XF_16UC1` | 通常为 `d×16`，含 4 位小数 | 除以 16 或调整下游公式/Q |
| OpenCV `StereoBM` | `CV_16S` | `d×16` | 除以 16 |
| OpenCV `StereoSGBM` | `CV_16S` | `d×16` | 除以 16 |
| AIE-ML SBM | `int16_t` | 需按该 AIE case 的插值/Host 合同解释 | 以 Host 重组与参考转换为准 |

不要为了“类型匹配”只做位宽转换，而忽略数值尺度。深度错误 16 倍通常就是这个原因。

<a id="section-13"></a>

## 13. Vitis SGBM 与 OpenCV StereoSGBM 不是同一个参数模型

Vitis `SemiGlobalBM` 与 OpenCV `cv::StereoSGBM` 都属于 SGM 家族，但不能认为它们只要设置相同 P1/P2 就会逐像素一致。

主要差异包括：

- Vitis 使用固定 5×5 Census + Hamming cost。
- Vitis 只支持 2/3/4 个聚合方向。
- Vitis 输出整数 8-bit 视差，没有 OpenCV 的 4-bit 亚像素格式。
- Vitis API 没有 `uniquenessRatio`、`speckleWindowSize`、`disp12MaxDiff` 等完整后处理参数。
- Vitis 的 `NDISP`、`PU`、方向数和图像上限是编译期参数。
- OpenCV 的多个模式和 Vitis `R=2/3/4` 不能简单一一映射。

因此，CPU 参考验证应优先使用仓库自带的 `xf_sgbm_tb.cpp` 软件模型；OpenCV StereoSGBM 更适合算法效果对照，而不是要求 bit-exact。

<a id="section-14"></a>

## 14. 官方建议的正确使用流程

### 14.1 算法/IP 级流程

1. 选择与 Vitis 工具版本匹配的 Vitis Libraries tag/branch。
2. 修改对应 example/test 的 `xf_config_params.h`。
3. 先跑 CSim，确认输入尺寸、左右顺序和视差方向。
4. 跑 CSynth，检查资源、目标时钟、II 和 latency。
5. 必要时跑 CoSim，验证 RTL 与 C 模型。
6. 导出 Vivado IP 或 `.xo`。

### 14.2 支持的标准平台

有匹配 `.xpfm` 和 XRT 环境时，使用 L2：

```text
HLS C++ kernel
  → v++ compile (.xo)
  → v++ link (.xclbin)
  → OpenCL/XRT Host
  → hw_emu
  → hw
```

### 14.3 自定义 Zynq/ZynqMP 板卡

没有官方 `.xpfm` 或 L2 allowlist 不支持时，使用：

```text
L1 HLS wrapper
  → CSim/CSynth/CoSim
  → Export Vivado IP
  → Vivado Block Design
  → HLS m_axi 接 PS DDR/HPC/HP
  → HLS s_axilite 接 PS 控制总线
  → 生成 bitstream/XSA
  → PS 端驱动、UIO、/dev/mem 或 XRT 控制
```

### 14.4 上板前验证清单

- 左右相机顺序是否正确。
- 校正后对应点是否基本位于同一水平线。
- 实际尺寸是否不超过 HLS 编译尺寸。
- 灰度图 stride 是否与连续 buffer 合同一致。
- AXI pointer 地址是否满足对齐和地址宽度要求。
- 输入/输出 buffer 字节数是否按真实像素类型计算。
- P1/P2 是否满足 `P1<P2<=100`。
- kernel 输出视差尺度是否与深度代码一致。
- `d=0`、遮挡、反光和低纹理区域是否被当作无效点处理。
- 深度公式中的焦距、基线和输出单位是否一致。
- kernel latency、DDR/cache 同步和端到端 latency 是否分别统计。

<a id="section-15"></a>

## 15. 当前 HXB/AXU5EV 工程与本分析仓库的版本关系

当前机器安装的是 Vitis/Vivado 2021.1，而本分析目录中的仓库是 `v2026.1_re`。两者不能直接混合作为同一构建环境。

当前 HXB 工程的 HLS 脚本 [`../hxb_cpp/fpga_sgbm/hls/run_hls.tcl`](../hxb_cpp/fpga_sgbm/hls/run_hls.tcl) 实际引用：

```text
/home/hcc/Desktop/HXB/Vitis_Libraries/vision
```

该目录是 2021.1 时代的 Vitis Libraries 源码，commit `f217398d...`，与 Vitis HLS 2021.1 匹配。当前项目并没有使用本文分析目录中的 2026.1 `xf_sgbm.hpp` 进行实际综合。

2021.1 与 2026.1 的 `xf_sgbm.hpp` 主要差异包括：

- 旧版 `#pragma HLS RESOURCE` 与新版 `bind_storage/BIND_OP`。
- 新版 `xf::cv::Mat` 增加显式 `XFCVDEPTH` 模板参数。
- 新版增加 dependence pragma。
- 新版 L1 Makefile/CLI 使用 `vitis-run` 和新版统一命令流。

因此当前 AXU5EV 工程应继续采用：

```text
Vitis/Vivado 2021.1
+ /home/hcc/Desktop/HXB/Vitis_Libraries（2021.1 版本）
+ 自定义 HLS IP/Vivado BD/PS runtime
```

若升级到 2026.1，应把工具链、库、平台、wrapper、Vivado 集成和板端 runtime 作为一个整体重新验证，而不是只替换一个头文件。

<a id="section-16"></a>

## 16. 当前 HXB SGBM 实现与官方结构的对应关系

当前工程已经完成了官方 L1 之后的自定义板级集成：

| HXB 文件 | 对应官方层次 | 功能 |
| --- | --- | --- |
| [`../hxb_cpp/fpga_sgbm/hls/xf_config_params.h`](../hxb_cpp/fpga_sgbm/hls/xf_config_params.h) | L1 config | 612×512、64 视差、PU32、4 方向 |
| [`../hxb_cpp/fpga_sgbm/hls/xf_sgbm_accel.cpp`](../hxb_cpp/fpga_sgbm/hls/xf_sgbm_accel.cpp) | L1 wrapper | DDR 数组与 `SemiGlobalBM` 的 HLS 顶层 |
| [`../hxb_cpp/fpga_sgbm/hls/run_hls.tcl`](../hxb_cpp/fpga_sgbm/hls/run_hls.tcl) | HLS 构建 | CSim、CSynth、CoSim、导出 IP |
| [`../hxb_cpp/fpga_sgbm/vivado/build_sgbm_accel_bd.tcl`](../hxb_cpp/fpga_sgbm/vivado/build_sgbm_accel_bd.tcl) | 自定义系统集成 | ZynqMP PS、AXI-Lite、DDR 与 HLS IP |
| [`../hxb_cpp/src/realtime_sgbm_hxb.cpp`](../hxb_cpp/src/realtime_sgbm_hxb.cpp) | PS runtime | 配置寄存器、DDR buffer、启动/等待 PL kernel |

当前配置：

```text
图像：612×512
输入：两幅 XF_8UC1
输出：一幅 XF_8UC1 整数视差
NDISP：64
PU：32
方向：4
目标器件：xczu5ev-sfvc784-2-i
```

已有综合记录见 [`../hxb_cpp/fpga_sgbm/hls/part_synthesis_matrix.md`](../hxb_cpp/fpga_sgbm/hls/part_synthesis_matrix.md)：ZU5EV 上 BRAM 和 LUT 占用已经较高。因此，把校正/Remap 也并入同一 PL pipeline 前必须先做新的资源预算。

<a id="section-17"></a>

## 17. 已发现的源码/文档注意事项

1. `depth3D` 与 `reprojectimageto3D` 的 RST 参数类型和当前 C++ 头文件不完全一致，应以源码断言、example config 和实际综合为准。
2. `3D_depth_float` testbench 对 OpenCV `×16` 视差没有做物理尺度归一化；它适合验证硬件与参考代码一致，不代表深度单位天然正确。
3. `reprojectimageto3D` 当前实现的 Q 和 XYZ 是定点/整数语义，不是通用 OpenCV double/float 版本。
4. `xf_reproject3D.hpp` 存在自身 include 的冗余写法，虽然被 include guard 阻止递归，但不应照搬为新的编码模式。
5. 当前 L3 `stereopipeline` 是 StereoBM，不是 SGBM。
6. SGBM L2 示例的平台 allowlist 不能代表所有 ZynqMP 自定义板都被官方验证。
7. `HEIGHT/WIDTH` 是硬件最大尺寸，实际输入不得超过；官方还建议 CoSim 时配置尺寸与实际测试图尺寸一致。
8. SGBM 的 P1/P2 是 8-bit 运行时参数，且 P2 最大为 100，与 OpenCV 中常见的更大 P1/P2 数字范围不同。

<a id="section-18"></a>

## 18. 推荐阅读顺序

若要从零读懂代码，推荐按以下顺序：

1. [`vision/README.md`](Vitis_Libraries/vision/README.md)：先理解 L1/L2/L3。
2. [`L1/examples/sgbm/config/xf_config_params.h`](Vitis_Libraries/vision/L1/examples/sgbm/config/xf_config_params.h)：理解硬件配置。
3. [`L1/examples/sgbm/xf_sgbm_accel.cpp`](Vitis_Libraries/vision/L1/examples/sgbm/xf_sgbm_accel.cpp)：理解 HLS 接口和 wrapper。
4. [`L1/include/imgproc/xf_sgbm.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_sgbm.hpp)：按 Census → cost → optimization → disparity 阅读。
5. [`L1/examples/sgbm/xf_sgbm_tb.cpp`](Vitis_Libraries/vision/L1/examples/sgbm/xf_sgbm_tb.cpp)：理解软件参考和输入输出。
6. [`L2/examples/sgbm/xf_sgbm_tb.cpp`](Vitis_Libraries/vision/L2/examples/sgbm/xf_sgbm_tb.cpp)：理解 XRT/OpenCL Host。
7. [`L1/include/imgproc/xf_stereolbm.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_stereolbm.hpp)：对比局部 BM 与 SGBM。
8. [`L1/examples/stereopipeline/xf_stereo_pipeline_accel.cpp`](Vitis_Libraries/vision/L1/examples/stereopipeline/xf_stereo_pipeline_accel.cpp)：理解校正 + BM 数据流。
9. [`L1/include/imgproc/xf_3ddepth.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_3ddepth.hpp) 和 [`xf_reproject3D.hpp`](Vitis_Libraries/vision/L1/include/imgproc/xf_reproject3D.hpp)：理解视差后的几何转换。
10. 最后再阅读 AIE-ML SBM，因为它使用 ADF graph、GMIO、MemTile 和 AIE 向量 API，架构与 PL HLS 不同。

<a id="section-19"></a>

## 19. 官方资料与仓库内文档

### 19.1 AMD 官方在线文档

- [Vitis Vision Library 2026.1](https://docs.amd.com/r/en-US/Vitis_Libraries/vision/index.html)
- [Semi Global Method for Stereo Disparity Estimation](https://docs.amd.com/r/en-US/Vitis_Libraries/vision/api-reference.html_1_98?contentId=HcwXOZV5_cMDsGgr614pSw)
- [Getting Started with HLS](https://docs.amd.com/r/en-US/Vitis_Libraries/vision/overview.html_3?contentId=SBpOnUFCei78vwsyo~LzTg)
- [Run a L1 Example](https://docs.amd.com/r/en-US/Vitis_Libraries/compilation_execution.html_1_0?contentId=PyNu3CkGIXdUK55oWphixw)
- [Run a L2 Example](https://docs.amd.com/r/en-US/Vitis_Libraries/compilation_execution.html_2_0?contentId=sESuxuOmJ0oXPAKUpVXc7Q)
- [Stereo Vision Pipeline](https://docs.amd.com/r/en-US/Vitis_Libraries/vision/overview.html_5_5?contentId=JcElQOUf3Vk0zC1ZkK~kOg)

### 19.2 当前仓库内的原始文档

- [`vision/docs/src/api-reference.rst`](Vitis_Libraries/vision/docs/src/api-reference.rst)
- [`vision/docs/src/design-examples.rst`](Vitis_Libraries/vision/docs/src/design-examples.rst)
- [`vision/docs/src/include/3Ddepth_api.rst`](Vitis_Libraries/vision/docs/src/include/3Ddepth_api.rst)
- [`vision/docs/src/include/3Dpoint_api.rst`](Vitis_Libraries/vision/docs/src/include/3Dpoint_api.rst)
- [`vision/L1/README.md`](Vitis_Libraries/vision/L1/README.md)
- [`vision/L2/README.md`](Vitis_Libraries/vision/L2/README.md)

<a id="section-20"></a>

## 20. 一句话总结

这个仓库提供的是一组分层的双目视觉硬件积木：`xf_stereo_pipeline.hpp + xf_remap.hpp` 负责校正，`xf_sgbm.hpp` 或 `xf_stereolbm.hpp` 负责生成视差，`xf_3ddepth.hpp` 或 `xf_reproject3D.hpp` 负责把视差转换为深度/XYZ；L1、L2、L3 表示不同开发和部署层级，而不是三份彼此独立的 SGBM 算法。
