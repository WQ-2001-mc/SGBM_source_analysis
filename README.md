# Vitis Stereo Lib code analysis

本仓库面向 AMD/Xilinx Vitis Vision 双目立体匹配开源代码，重点分析两类硬件算法：基于局部 SAD 代价的 SAD_BM（Vitis API 中对应 `StereoBM` / `SADBlockMatching`）和半全局块匹配 SGBM（Vitis API 中对应 `SemiGlobalBM`）。除源码调用链与 HLS 微架构外，仓库还整理了双目相机参数、PS/PL 数据流、片上存储、外部带宽、资源瓶颈及 1080p30 场景下的 FPGA 工程测算。

## 两类核心算法

| 算法 | Vitis 中的主要名称 | 本仓库分析重点 |
|---|---|---|
| SAD_BM | `StereoBM`、`SADBlockMatching` | Sobel 预处理、滑窗 SAD、分组视差并行、唯一性/纹理过滤、亚像素输出与资源模型 |
| SGBM | `SemiGlobalBM` | Census、Hamming 代价、四路径 SGM 聚合、WTA、路径状态存储、吞吐与资源瓶颈 |

两种算法都是本仓库的核心内容；仓库名称中的 “Stereo Lib” 指向 Vitis Vision 双目立体匹配代码整体，而不是只指其中一种算法。

## 仓库内容

| 文件或目录 | 内容 |
|---|---|
| [`Vitis_Libraries_双目立体匹配源码结构分析.md`](Vitis_Libraries_双目立体匹配源码结构分析.md) | Vitis Libraries 双目深度模块总览，覆盖 SGBM、SAD_BM/StereoBM、双目校正、视差转深度/XYZ、示例层级和源码结构。 |
| [`vitis算法分析/SGBM算法分析.md`](vitis算法分析/SGBM算法分析.md) | Vitis Vision SGBM 的 Census、Hamming 代价、四路径聚合、WTA、吞吐、存储和瓶颈分析。 |
| [`vitis算法分析/SAD_BM算法分析.md`](vitis算法分析/SAD_BM算法分析.md) | Vitis Vision SAD_BM/StereoBM 的预处理、滑窗 SAD、视差并行、亚像素输出和性能模型。 |
| [`深度相机fpga测算.md`](深度相机fpga测算.md) | 双目 IR 相机建模、基线和视差范围评估，以及 Matlab/Vitis 方案在 1080p30 下的 FPGA 资源、带宽和器件选型测算。 |
| [`sgbm算子整理.xlsx`](sgbm算子整理.xlsx) | SGBM 算子和相关信息的表格化整理。 |
| `Vitis_Libraries/` | AMD/Xilinx Vitis Libraries 子模块，用于定位和复核分析中引用的开源代码。 |
| `vitis算法分析/figures/` | SAD_BM 与 SGBM 的 FPGA 架构图及其可编辑绘图脚本。 |
| `*.svg`、`*.png` | 文档引用的相机模型、视差、资源换算和流程示意图。 |

## 推荐阅读顺序

1. 阅读 [`Vitis_Libraries_双目立体匹配源码结构分析.md`](Vitis_Libraries_双目立体匹配源码结构分析.md)，建立两类算法、源码层级和端到端数据流的整体认识。
2. 阅读 [`vitis算法分析/SAD_BM算法分析.md`](vitis算法分析/SAD_BM算法分析.md) 和 [`vitis算法分析/SGBM算法分析.md`](vitis算法分析/SGBM算法分析.md)，分别查看局部块匹配与半全局块匹配的实现细节、性能和瓶颈。
3. 阅读 [`深度相机fpga测算.md`](深度相机fpga测算.md)，将相机参数、视差范围、吞吐和 FPGA 资源联系起来。
4. 使用 [`sgbm算子整理.xlsx`](sgbm算子整理.xlsx) 快速查阅 SGBM 专项算子信息。

## 获取源码

仓库使用 Git 子模块固定 Vitis Libraries 源码版本。建议克隆时同时初始化子模块：

```bash
git clone --recurse-submodules https://github.com/WQ-2001-mc/Vitis-Stereo-Lib-code-analysis.git
```

如果已经完成普通克隆，可在仓库目录中执行：

```bash
git submodule update --init --recursive
```

当前 `Vitis_Libraries` 子模块固定在提交 `629b2c979f65561f07e4e87b860f306cb480895e`。子模块中的第三方开源代码遵循其自身许可证。

## 使用说明

- 文档中的吞吐、LUT/ALM、BRAM/URAM、DSP 和接口带宽数据用于架构评估，不替代目标器件上的 HLS 综合、实现和时序报告。
- 资源结果会随图像尺寸、最大视差、路径数、窗口尺寸、像素并行度、器件系列和工具版本变化。
- 接入实际双目系统前，需要结合相机标定、极线校正、输入位宽、帧同步、DDR 布局和板级接口重新验证。
