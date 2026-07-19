# SGBM 源码分析与 FPGA 性能测算

本仓库整理了 AMD/Xilinx Vitis Vision 双目立体匹配相关源码分析，并围绕 StereoBM、Semi-Global Block Matching（SGBM）、双目相机参数和 FPGA 资源选型给出性能测算。内容覆盖算法调用链、PS/PL 数据流、HLS 流水线、片上存储与外部带宽、资源瓶颈，以及 1080p30 场景下的工程估算。

## 仓库内容

| 文件或目录 | 内容 |
|---|---|
| [`Vitis_Libraries_双目深度与SGBM源码结构分析.md`](Vitis_Libraries_双目深度与SGBM源码结构分析.md) | Vitis Libraries 中双目深度相关模块的总览，涵盖 SGBM、StereoBM、双目校正、视差转深度/XYZ、示例层级和源码结构。 |
| [`深度相机fpga测算.md`](深度相机fpga测算.md) | 双目 IR 相机建模、基线和视差范围评估，以及 Matlab/Vitis SGBM 在 1080p30 下的 FPGA 资源、带宽和器件选型测算。 |
| [`vitis算法分析/SGBM算法分析.md`](vitis算法分析/SGBM算法分析.md) | Vitis Vision SGBM 的 Census、Hamming 代价、四路径聚合、WTA、吞吐、存储和瓶颈分析。 |
| [`vitis算法分析/SAD_BM算法分析.md`](vitis算法分析/SAD_BM算法分析.md) | Vitis Vision StereoBM/SADBlockMatching 的预处理、滑窗 SAD、视差并行、亚像素输出和性能模型。 |
| [`sgbm算子整理.xlsx`](sgbm算子整理.xlsx) | SGBM 算子和相关信息的表格化整理。 |
| `Vitis_Libraries/` | AMD/Xilinx Vitis Libraries 子模块，用于定位和复核分析中引用的源码。 |
| `*.svg`、`*.png` | 文档引用的相机模型、视差、资源换算和流程示意图。 |

## 推荐阅读顺序

1. 阅读 [`Vitis_Libraries_双目深度与SGBM源码结构分析.md`](Vitis_Libraries_双目深度与SGBM源码结构分析.md)，建立源码结构和端到端数据流的整体认识。
2. 阅读 [`vitis算法分析/SGBM算法分析.md`](vitis算法分析/SGBM算法分析.md) 和 [`vitis算法分析/SAD_BM算法分析.md`](vitis算法分析/SAD_BM算法分析.md)，查看两类匹配算法的实现细节和瓶颈。
3. 阅读 [`深度相机fpga测算.md`](深度相机fpga测算.md)，将相机参数、视差范围、吞吐和 FPGA 资源联系起来。
4. 使用 [`sgbm算子整理.xlsx`](sgbm算子整理.xlsx) 快速查阅算子信息。

## 获取源码

仓库使用 Git 子模块固定 Vitis Libraries 源码版本。建议克隆时同时初始化子模块：

```bash
git clone --recurse-submodules https://github.com/WQ-2001-mc/SGBM_source_analysis.git
```

如果已经完成普通克隆，可在仓库目录中执行：

```bash
git submodule update --init --recursive
```

当前 `Vitis_Libraries` 子模块固定在提交 `629b2c979f65561f07e4e87b860f306cb480895e`。子模块中的第三方源码遵循其自身许可证。

## 使用说明

- 文档中的吞吐、LUT/ALM、BRAM/URAM、DSP 和接口带宽数据用于架构评估，不替代目标器件上的 HLS 综合、实现和时序报告。
- 资源结果会随图像尺寸、最大视差、路径数、窗口尺寸、像素并行度、器件系列和工具版本变化。
- 接入实际双目系统前，需要结合相机标定、极线校正、输入位宽、帧同步、DDR 布局和板级接口重新验证。
