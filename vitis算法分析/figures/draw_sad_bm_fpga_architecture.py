#!/usr/bin/env python3
"""Draw a source-auditable Vitis Vision SAD-BM FPGA architecture figure."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle


# Editable vector text and a Chinese-capable sans-serif stack.
plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans", "sans-serif"],
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
    }
)

CHINESE_FONT = Path("/System/Library/Fonts/STHeiti Medium.ttc")
if CHINESE_FONT.exists():
    font_manager.fontManager.addfont(str(CHINESE_FONT))
    plt.rcParams["font.sans-serif"] = ["Heiti TC", "Arial", "DejaVu Sans", "sans-serif"]


OUT_DIR = Path(__file__).resolve().parent
OUT_STEM = OUT_DIR / "sad_bm_fpga_architecture"

# Repository default configuration.
WIDTH = 1920
HEIGHT = 1080
WSIZE = 11
NDISP = 32
PARALLEL_UNITS = 32
BUF_SIZE = WIDTH + WSIZE - 1
RWIN_WIDTH = WSIZE + PARALLEL_UNITS - 1
SWEEPS = NDISP // PARALLEL_UNITS

# Source-declared logical storage capacities. They are not post-synthesis BRAM counts.
SOBEL_LINE_BITS_EACH = 3 * WIDTH * 8
BM_LINE_BITS = 2 * WSIZE * BUF_SIZE * 8
WINDOW_BITS = (WSIZE * WSIZE + WSIZE * RWIN_WIDTH) * 8
SAD_STATE_BITS = PARALLEL_UNITS * 32 + PARALLEL_UNITS * WSIZE * 16
CROSS_SWEEP_BITS = 6 * BUF_SIZE * 32 + BUF_SIZE * 16 + BUF_SIZE

assert BUF_SIZE == 1930
assert RWIN_WIDTH == 42
assert SWEEPS == 1
assert SOBEL_LINE_BITS_EACH == 46080
assert BM_LINE_BITS == 339680
assert WINDOW_BITS == 4664
assert SAD_STATE_BITS == 6656
assert CROSS_SWEEP_BITS == 403370


COLORS = {
    "ink": "#203047",
    "muted": "#5A6978",
    "line": "#A9B5C1",
    "blue": "#2767A6",
    "blue_fill": "#E5F0FA",
    "teal": "#238B87",
    "teal_fill": "#DFF3F1",
    "violet": "#715FB0",
    "violet_fill": "#ECE9F8",
    "orange": "#C97525",
    "orange_fill": "#FAECDD",
    "green": "#2F8257",
    "green_fill": "#E3F2E8",
    "red": "#AF4545",
    "red_fill": "#F8E5E5",
    "gray_fill": "#EFF3F6",
    "white": "#FFFFFF",
}


def rounded_box(
    ax,
    x,
    y,
    w,
    h,
    title,
    lines=(),
    *,
    edge,
    face,
    title_size=11.5,
    line_size=9.6,
    line_gap=3.05,
    title_pad=2.0,
    lw=1.55,
    align="left",
    zorder=2,
):
    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.34,rounding_size=0.9",
        linewidth=lw,
        edgecolor=edge,
        facecolor=face,
        zorder=zorder,
    )
    ax.add_patch(patch)
    ha = "center" if align == "center" else "left"
    tx = x + w / 2 if align == "center" else x + 1.35
    ax.text(
        tx,
        y + h - title_pad,
        title,
        ha=ha,
        va="top",
        fontsize=title_size,
        fontweight="bold",
        color=edge,
        zorder=zorder + 1,
    )
    for idx, line in enumerate(lines):
        ax.text(
            tx,
            y + h - title_pad - 3.85 - idx * line_gap,
            line,
            ha=ha,
            va="top",
            fontsize=line_size,
            color=COLORS["ink"],
            zorder=zorder + 1,
        )
    return patch


def data_arrow(
    ax,
    start,
    end,
    label=None,
    *,
    color=None,
    lw=2.0,
    label_offset=(0.0, 1.0),
    connection="arc3,rad=0",
    zorder=6,
):
    color = color or COLORS["blue"]
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=14,
        linewidth=lw,
        color=color,
        connectionstyle=connection,
        shrinkA=1.5,
        shrinkB=1.5,
        zorder=zorder,
    )
    ax.add_patch(arrow)
    if label:
        mx = (start[0] + end[0]) / 2 + label_offset[0]
        my = (start[1] + end[1]) / 2 + label_offset[1]
        ax.text(
            mx,
            my,
            label,
            ha="center",
            va="bottom",
            fontsize=8.8,
            fontweight="bold",
            color=color,
            zorder=zorder + 1,
        )


def control_arrow(ax, start, end, label=None, *, connection="arc3,rad=0"):
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=12,
        linewidth=1.4,
        color=COLORS["muted"],
        linestyle=(0, (3, 3)),
        connectionstyle=connection,
        shrinkA=1.5,
        shrinkB=1.5,
        zorder=5,
    )
    ax.add_patch(arrow)
    if label:
        ax.text(
            (start[0] + end[0]) / 2,
            (start[1] + end[1]) / 2 + 0.8,
            label,
            ha="center",
            va="bottom",
            fontsize=8.5,
            color=COLORS["muted"],
            zorder=6,
        )


def panel_title(ax, x, y, letter, title):
    ax.text(x, y, letter, fontsize=14.5, fontweight="bold", color=COLORS["ink"], ha="left", va="top")
    ax.text(x + 3.0, y, title, fontsize=13.0, fontweight="bold", color=COLORS["ink"], ha="left", va="top")


def draw_metric_table(ax, x, y, w, h):
    rounded_box(
        ax,
        x,
        y,
        w,
        h,
        "U200 整核实现报告（不是子模块拆分）",
        (),
        edge=COLORS["red"],
        face=COLORS["red_fill"],
        title_size=11.4,
    )
    labels = ["LUT", "FF", "BRAM", "DSP"]
    values = ["19,380", "20,670", "13", "7"]
    cell_w = (w - 3.0) / 4
    cell_y = y + h - 10.5
    for idx, (label, value) in enumerate(zip(labels, values)):
        cx = x + 1.5 + idx * cell_w
        ax.add_patch(
            Rectangle(
                (cx, cell_y - 7.0),
                cell_w - 0.5,
                7.0,
                facecolor="white",
                edgecolor="#D8B7B7",
                linewidth=0.85,
                zorder=3,
            )
        )
        ax.text(cx + (cell_w - 0.5) / 2, cell_y - 1.7, label, ha="center", va="center", fontsize=8.8, color=COLORS["muted"], fontweight="bold", zorder=4)
        ax.text(cx + (cell_w - 0.5) / 2, cell_y - 4.9, value, ha="center", va="center", fontsize=13.2, color=COLORS["red"], fontweight="bold", zorder=4)
    ax.text(x + 1.5, y + 6.4, "FHD·NPPC1·PU=32·D=32·300 MHz  →  135 FPS", fontsize=9.4, color=COLORS["ink"], ha="left", va="center")
    ax.text(x + 1.5, y + 2.2, "报告只给出 kernel 总量，无法准确倒推各框的 LUT / BRAM 占用。", fontsize=8.8, color=COLORS["red"], ha="left", va="center", fontweight="bold")


def draw_comparator_tree(ax, x, y, w, h):
    """Compact visual for the exact 32-input recursive xFMinSAD tree."""
    levels = [32, 16, 8, 4, 2, 1]
    xs = [x + 1.5 + i * (w - 3.0) / (len(levels) - 1) for i in range(len(levels))]
    for idx, (count, cx) in enumerate(zip(levels, xs)):
        bar_h = max(1.2, h * count / 40)
        cy = y + h / 2
        ax.add_patch(
            FancyBboxPatch(
                (cx - 0.85, cy - bar_h / 2),
                1.7,
                bar_h,
                boxstyle="round,pad=0.05,rounding_size=0.25",
                facecolor=COLORS["green_fill"] if count > 1 else COLORS["green"],
                edgecolor=COLORS["green"],
                linewidth=1.0,
                zorder=4,
            )
        )
        ax.text(cx, y + 0.25, str(count), ha="center", va="bottom", fontsize=8.0, color=COLORS["green"], fontweight="bold", zorder=5)
        if idx < len(xs) - 1:
            data_arrow(ax, (cx + 0.95, cy), (xs[idx + 1] - 0.95, cy), color=COLORS["green"], lw=1.1)


fig, ax = plt.subplots(figsize=(20, 13.6))
fig.patch.set_facecolor("white")
ax.set_xlim(0, 180)
ax.set_ylim(0, 120)
ax.set_aspect("equal")
ax.axis("off")

# Title and scope contract.
ax.text(2, 117.2, "Vitis Vision SAD-BM / StereoBM FPGA IP 核与电路数据流", fontsize=22.0, fontweight="bold", color=COLORS["ink"], ha="left", va="top")
ax.text(2, 113.0, "默认源码配置：1920×1080 · XF_8UC1 → XF_16UC1 (Q12.4) · WSIZE=11 · D=32 · PU=32 · NPPC1 · USE_URAM=0 · sweep=1", fontsize=11.0, color=COLORS["muted"], ha="left", va="top")
ax.text(178, 113.0, "结构尺度：源码可证  |  器件资源：仓库 U200 报告", fontsize=10.0, color=COLORS["muted"], ha="right", va="top")

# -----------------------------------------------------------------------------
# Panel a: complete HLS kernel dataflow.
panel_title(ax, 2.0, 108.9, "a", "整核数据流：stereolbm_accel()  /  HLS DATAFLOW")
kernel = FancyBboxPatch(
    (1.5, 77.2),
    177,
    28.6,
    boxstyle="round,pad=0.38,rounding_size=1.1",
    linewidth=1.5,
    edgecolor="#8996A3",
    facecolor="#FBFCFE",
    linestyle=(0, (5, 3)),
    zorder=0,
)
ax.add_patch(kernel)

rounded_box(ax, 3.0, 80.8, 13.0, 20.5, "DDR / AXI4", ("gmem0：左图", "gmem1：右图", "gmem2：4 B 状态", "gmem3：视差图"), edge=COLORS["ink"], face=COLORS["gray_fill"], title_size=11.4, line_size=9.2, line_gap=3.2)

rounded_box(ax, 19.2, 93.3, 14.0, 8.0, "Array2xfMat", ("左8-bit · 1 px/clk",), edge=COLORS["blue"], face=COLORS["blue_fill"], title_size=10.6, line_size=8.7, line_gap=2.5)
rounded_box(ax, 19.2, 81.0, 14.0, 8.0, "Array2xfMat", ("右8-bit · 1 px/clk",), edge=COLORS["blue"], face=COLORS["blue_fill"], title_size=10.6, line_size=8.7, line_gap=2.5)

rounded_box(ax, 36.7, 91.0, 25.0, 12.1, "左预处理：Sobel 3×3 + clip", ("Gx/Gy：16-bit；仅 Gx 保留", "3×1920×8-bit = 46,080 bit", "Gx 截断平移后：8-bit"), edge=COLORS["teal"], face=COLORS["teal_fill"], title_size=10.7, line_size=8.6, line_gap=2.6)
rounded_box(ax, 36.7, 78.1, 25.0, 12.1, "右预处理：Sobel 3×3 + clip", ("Gx/Gy：16-bit；仅 Gx 保留", "3×1920×8-bit = 46,080 bit", "Gx 截断平移后：8-bit"), edge=COLORS["teal"], face=COLORS["teal_fill"], title_size=10.7, line_size=8.6, line_gap=2.6)

rounded_box(ax, 66.1, 80.1, 68.0, 21.3, "xFSADBlockMatching（下图放大）", ("11×11 滑窗  →  32 路增量 SAD  →  WTA  →  纹理/唯一性/边界过滤  →  亚像素", "列主流水目标 II=1；sweep=D/PU=1；峰值 1 个输出像素/周期", "输入：左/右 8-bit clipped stream     输出：16-bit Q12.4 disparity stream"), edge=COLORS["violet"], face=COLORS["violet_fill"], title_size=12.3, line_size=10.0, line_gap=3.5, lw=1.9)

rounded_box(ax, 138.2, 86.0, 17.2, 10.0, "xfMat2Array", ("16-bit · 1 px/clk", "写回 gmem3"), edge=COLORS["green"], face=COLORS["green_fill"], title_size=10.9, line_size=8.9, line_gap=2.7)
rounded_box(ax, 159.0, 82.9, 16.8, 15.0, "输出 DDR", ("XF_16UC1", "Q12.4", "raw / 16 = disparity", "无效值 = 0"), edge=COLORS["ink"], face=COLORS["gray_fill"], title_size=11.0, line_size=8.7, line_gap=2.55)

data_arrow(ax, (16.0, 97.2), (19.2, 97.2))
data_arrow(ax, (16.0, 85.0), (19.2, 85.0))
data_arrow(ax, (33.2, 97.2), (36.7, 97.2), "8-bit", label_offset=(0, 0.8))
data_arrow(ax, (33.2, 85.0), (36.7, 85.0), "8-bit", label_offset=(0, 0.8))
data_arrow(ax, (61.7, 97.0), (66.1, 95.9), "8-bit Gx'", label_offset=(0, 1.15), connection="arc3,rad=-0.08")
data_arrow(ax, (61.7, 84.1), (66.1, 85.6), "8-bit Gx'", label_offset=(0, -2.2), connection="arc3,rad=0.08")
data_arrow(ax, (134.1, 91.1), (138.2, 91.1), "16-bit", color=COLORS["green"], label_offset=(0, 0.8))
data_arrow(ax, (155.4, 91.1), (159.0, 91.1), color=COLORS["green"])

rounded_box(ax, 82.0, 102.4, 40.0, 2.6, "AXI4-Lite：rows · cols    |    bm_state：cap · uniqueness · texture · minDisp", (), edge=COLORS["muted"], face=COLORS["gray_fill"], title_size=8.7, title_pad=0.65, align="center", lw=1.0)
control_arrow(ax, (102.0, 102.3), (102.0, 99.9), "尺寸 / 阈值")

# -----------------------------------------------------------------------------
# Panel b: xFSADBlockMatching circuit structure.
panel_title(ax, 2.0, 73.9, "b", "xFSADBlockMatching 电路结构（默认 sweep=1）")
core = FancyBboxPatch(
    (1.5, 34.5),
    177,
    36.6,
    boxstyle="round,pad=0.38,rounding_size=1.1",
    linewidth=1.7,
    edgecolor=COLORS["violet"],
    facecolor="#FCFBFE",
    zorder=0,
)
ax.add_patch(core)

rounded_box(ax, 3.2, 42.0, 24.6, 24.2, "左/右行缓存", ("2 × [11 × 1930 × 8-bit]", "= 339,680 bit", "= 41.46 KiB 原始容量", "", "行维完全分割", "读出 11 行列向量", "边界填充 cap"), edge=COLORS["orange"], face=COLORS["orange_fill"], title_size=11.2, line_size=9.2, line_gap=2.75)

rounded_box(ax, 31.0, 51.4, 23.0, 14.8, "滑动窗口寄存器", ("左：11×11×8 = 968 bit", "右：11×42×8 = 3,696 bit", "合计：4,664 bit / 583 B", "两维完全分割"), edge=COLORS["teal"], face=COLORS["teal_fill"], title_size=11.0, line_size=9.0, line_gap=2.7)
rounded_box(ax, 31.0, 36.8, 23.0, 11.5, "纹理统计", ("xFUpdateTextureSum", "11 行展开更新", "textureThreshold 过滤"), edge=COLORS["blue"], face=COLORS["blue_fill"], title_size=10.7, line_size=8.9, line_gap=2.4)

rounded_box(ax, 58.0, 45.2, 39.0, 21.0, "32 路增量 SAD 处理阵列", ("d = 0…31；每路新列含 11 个 |L−R|", "II=1 每列逻辑工作量：32×11 = 352 abs-diff", "b_sum − a_sum 更新整个 11×11 SAD", "sad[32]：32-bit  →  1,024 bit", "sad_cols[32][11]：16-bit  →  5,632 bit", "两数组均完全分割；合计 6,656 bit"), edge=COLORS["violet"], face=COLORS["violet_fill"], title_size=11.6, line_size=9.1, line_gap=2.55, lw=1.9)

rounded_box(ax, 58.0, 35.3, 39.0, 7.1, "列主流水调度", ("2,103,700 次 / 帧；#pragma pipeline II=1",), edge=COLORS["muted"], face=COLORS["gray_fill"], title_size=10.2, line_size=8.7, line_gap=2.3)

rounded_box(ax, 101.0, 48.2, 22.0, 18.0, "WTA：xFMinSAD<32>", ("32 输入递归比较树", "31 节点；5 级；输出 min+d"), edge=COLORS["green"], face=COLORS["green_fill"], title_size=11.0, line_size=9.0, line_gap=2.7)
draw_comparator_tree(ax, 103.0, 49.4, 18.0, 6.1)

rounded_box(ax, 127.0, 48.2, 22.0, 18.0, "过滤与唯一性", ("低纹理：texture sum", "几何边界：窗口 + D", "唯一性：最多 32 候选检查", "排除 d* ± 1", "失败则输出 0"), edge=COLORS["red"], face=COLORS["red_fill"], title_size=11.0, line_size=9.0, line_gap=2.6)

rounded_box(ax, 153.0, 48.2, 22.0, 18.0, "亚像素 + 输出", ("p / min / n 定点拟合", "带符号定点除法", "delta：10-bit", "out：16-bit Q12.4", "1 disparity / clk"), edge=COLORS["blue"], face=COLORS["blue_fill"], title_size=11.0, line_size=9.0, line_gap=2.6)

data_arrow(ax, (27.8, 58.0), (31.0, 58.0), color=COLORS["orange"])
data_arrow(ax, (54.0, 58.0), (58.0, 58.0), color=COLORS["teal"])
data_arrow(ax, (97.0, 58.0), (101.0, 58.0), "32× SAD", color=COLORS["violet"], label_offset=(0, 0.9))
data_arrow(ax, (123.0, 58.0), (127.0, 58.0), "min + d", color=COLORS["green"], label_offset=(0, 0.9))
data_arrow(ax, (149.0, 58.0), (153.0, 58.0), "valid / p,m,n", color=COLORS["red"], label_offset=(0, 0.9))
data_arrow(ax, (42.5, 51.4), (42.5, 48.3), color=COLORS["blue"], lw=1.4)

rounded_box(ax, 101.0, 35.5, 74.0, 9.3, "跨 sweep 每列状态（默认 sweep=1，后续读分支不执行）", ("6×1930×32-bit + 1930×16-bit + 1930×1-bit = 403,370 bit = 49.24 KiB 声明容量", "minsad / skip_val / edge_neighbor / edge / minsad_p / minsad_n + mind + skip；实现可被 HLS 优化"), edge=COLORS["orange"], face=COLORS["orange_fill"], title_size=10.4, line_size=8.7, line_gap=2.35)
data_arrow(ax, (112.0, 48.2), (112.0, 44.8), color=COLORS["orange"], lw=1.35)
data_arrow(ax, (166.0, 44.8), (166.0, 48.2), color=COLORS["orange"], lw=1.35)

# -----------------------------------------------------------------------------
# Panel c: evidence and interpretation boundaries.
panel_title(ax, 2.0, 31.6, "c", "器件规模与解读边界")

rounded_box(ax, 2.0, 3.2, 55.0, 25.4, "源码可精确数出的主要结构", ("• 预处理：2 套 Sobel 3×3 + clip；每套 3 行缓存", "  两套 Sobel 行缓存原始容量合计 92,160 bit", "• BM 行缓存：339,680 bit；滑窗：4,664 bit", "• 32 路 SAD 主状态：6,656 bit", "• WTA：32 输入、31 比较节点、5 级树", "• 逻辑像素工作量：最多 352 个 abs-diff/列周期"), edge=COLORS["teal"], face=COLORS["teal_fill"], title_size=11.2, line_size=9.2, line_gap=3.0)

draw_metric_table(ax, 61.0, 3.2, 55.0, 25.4)

rounded_box(ax, 120.0, 3.2, 58.0, 25.4, "不应误读为“精确子模块器件数”", ("• 数组 bit 容量 ≠ BRAM 块数；HLS 会映射到", "  BRAM / LUTRAM / FF，也可消除无效状态", "• “352 abs-diff/列”是 II=1 下的逻辑工作量，", "  实体共享、重定时和关键路径以 csynth 为准", "• 仓库未随附当前配置的层次化 csynth 报告，", "  所以只能准确标注整核总资源", "• minDisparity 虽传入，当前核心实际仅搜索 0…31"), edge=COLORS["red"], face=COLORS["red_fill"], title_size=11.2, line_size=9.2, line_gap=2.95)

# Compact legend and provenance.
data_arrow(ax, (3.0, 1.25), (8.0, 1.25), color=COLORS["blue"], lw=1.8)
ax.text(8.8, 1.25, "主数据流", fontsize=8.7, color=COLORS["muted"], va="center")
data_arrow(ax, (24.0, 1.25), (29.0, 1.25), color=COLORS["orange"], lw=1.5)
ax.text(29.8, 1.25, "状态 / 缓存反馈", fontsize=8.7, color=COLORS["muted"], va="center")
control_arrow(ax, (50.5, 1.25), (55.5, 1.25))
ax.text(56.3, 1.25, "控制 / 阈值", fontsize=8.7, color=COLORS["muted"], va="center")
ax.text(178, 1.25, "依据：xf_stereolbm_accel.cpp · xf_stereolbm.hpp · xf_sobel.hpp · xf_config_params.h · stereolbm-bm.rst", fontsize=8.0, color=COLORS["muted"], ha="right", va="center")

fig.subplots_adjust(left=0.012, right=0.988, top=0.988, bottom=0.012)
fig.savefig(f"{OUT_STEM}.svg", bbox_inches="tight", facecolor="white")
fig.savefig(f"{OUT_STEM}.pdf", bbox_inches="tight", facecolor="white")
fig.savefig(f"{OUT_STEM}.png", dpi=220, bbox_inches="tight", facecolor="white")

print(f"Wrote {OUT_STEM}.svg")
print(f"Wrote {OUT_STEM}.pdf")
print(f"Wrote {OUT_STEM}.png")
