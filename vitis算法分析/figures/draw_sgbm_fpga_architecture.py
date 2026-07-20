#!/usr/bin/env python3
"""Draw a source-auditable Vitis Vision SGBM FPGA architecture figure."""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Rectangle


# Required publication settings: keep SVG text editable.
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'DejaVu Sans', 'Liberation Sans']
plt.rcParams['svg.fonttype'] = 'none'
plt.rcParams['pdf.fonttype'] = 42

CHINESE_FONT = Path('/System/Library/Fonts/STHeiti Medium.ttc')
if CHINESE_FONT.exists():
    font_manager.fontManager.addfont(str(CHINESE_FONT))
    plt.rcParams['font.sans-serif'] = ['Heiti TC', 'Arial', 'DejaVu Sans', 'Liberation Sans']


OUT_DIR = Path(__file__).resolve().parent
OUT_STEM = OUT_DIR / 'sgbm_fpga_architecture'

COLORS = {
    'ink': '#203047',
    'muted': '#5B6B7C',
    'line': '#AAB6C2',
    'panel': '#F7F9FC',
    'blue': '#2F6FB0',
    'blue_fill': '#E5F0FA',
    'teal': '#238B87',
    'teal_fill': '#DFF3F1',
    'violet': '#7464B3',
    'violet_fill': '#ECE9F8',
    'orange': '#D57A24',
    'orange_fill': '#FAECDD',
    'green': '#36845B',
    'green_fill': '#E4F2E9',
    'red': '#B64A4A',
    'red_fill': '#F8E6E6',
    'gray_fill': '#EEF2F5',
    'white': '#FFFFFF',
}


def rounded_box(ax, x, y, w, h, title, lines=(), *, edge, face, title_size=10.2,
                line_size=8.2, lw=1.5, radius=0.9, title_color=None,
                align='left', title_y_pad=2.1, line_gap=2.65, zorder=2):
    patch = FancyBboxPatch(
        (x, y), w, h,
        boxstyle=f'round,pad=0.30,rounding_size={radius}',
        linewidth=lw, edgecolor=edge, facecolor=face, zorder=zorder,
    )
    ax.add_patch(patch)
    ha = 'center' if align == 'center' else 'left'
    tx = x + w / 2 if align == 'center' else x + 1.3
    ax.text(tx, y + h - title_y_pad, title, ha=ha, va='top', fontsize=title_size,
            fontweight='bold', color=title_color or edge, zorder=zorder + 1)
    for i, line in enumerate(lines):
        ax.text(tx, y + h - title_y_pad - 3.35 - i * line_gap, line,
                ha=ha, va='top', fontsize=line_size, color=COLORS['ink'],
                zorder=zorder + 1)
    return patch


def data_arrow(ax, start, end, label=None, *, color=None, lw=2.1,
               label_offset=(0, 1.25), connection='arc3,rad=0', zorder=5):
    color = color or COLORS['blue']
    arrow = FancyArrowPatch(
        start, end, arrowstyle='-|>', mutation_scale=13, linewidth=lw,
        color=color, connectionstyle=connection, shrinkA=1.5, shrinkB=1.5,
        zorder=zorder,
    )
    ax.add_patch(arrow)
    if label:
        mx = (start[0] + end[0]) / 2 + label_offset[0]
        my = (start[1] + end[1]) / 2 + label_offset[1]
        ax.text(mx, my, label, ha='center', va='bottom', fontsize=7.4,
                color=color, fontweight='bold', zorder=zorder + 1)
    return arrow


def control_arrow(ax, start, end, label=None):
    arrow = FancyArrowPatch(
        start, end, arrowstyle='-|>', mutation_scale=11, linewidth=1.35,
        color=COLORS['muted'], linestyle=(0, (3, 3)),
        connectionstyle='arc3,rad=0', zorder=4,
    )
    ax.add_patch(arrow)
    if label:
        ax.text((start[0] + end[0]) / 2, (start[1] + end[1]) / 2 + 0.8,
                label, ha='center', va='bottom', fontsize=7.2,
                color=COLORS['muted'], zorder=5)


def panel_title(ax, x, y, letter, title):
    ax.text(x, y, letter, fontsize=12.5, fontweight='bold', color=COLORS['ink'],
            ha='left', va='top')
    ax.text(x + 2.7, y, title, fontsize=11.0, fontweight='bold', color=COLORS['ink'],
            ha='left', va='top')


def draw_path_icon(ax, x, y):
    """Show the four raster-causal predecessors used by the source implementation."""
    center = (x + 8.1, y + 4.0)
    preds = [
        ((x + 2.0, y + 4.0), 'r0  ←'),
        ((x + 3.2, y + 8.0), 'r1  ↖'),
        ((x + 8.1, y + 8.5), 'r2  ↑'),
        ((x + 13.0, y + 8.0), 'r3  ↗'),
    ]
    for pos, label in preds:
        ax.add_patch(Circle(pos, 0.62, facecolor=COLORS['white'],
                            edgecolor=COLORS['violet'], linewidth=1.2, zorder=5))
        data_arrow(ax, pos, center, color=COLORS['violet'], lw=1.2, zorder=4)
        dy = 1.45 if pos[1] > center[1] else -1.55
        ax.text(pos[0], pos[1] + dy, label, ha='center', va='center', fontsize=6.8,
                color=COLORS['violet'], fontweight='bold', zorder=6)
    ax.add_patch(Circle(center, 0.82, facecolor=COLORS['violet'],
                        edgecolor=COLORS['violet'], linewidth=1.2, zorder=6))
    ax.text(center[0], center[1], 'p', ha='center', va='center', fontsize=8.3,
            color='white', fontweight='bold', zorder=7)
    ax.text(x + 8.1, y + 0.55, '当前像素 p 的 4 个因果前驱方向', ha='center',
            va='bottom', fontsize=7.2, color=COLORS['muted'])


def draw_resource_table(ax, x, y, w, h):
    rounded_box(ax, x, y, w, h, '仓库 API 文档：整核资源估算', (),
                edge=COLORS['red'], face=COLORS['red_fill'], title_size=10.2)
    labels = ['LUT', 'FF', 'BRAM_18K', 'DSP48E']
    values = ['19,102', '11,856', '205', '141']
    cell_y = y + h - 8.4
    cell_w = (w - 3.0) / 4
    for i, (label, value) in enumerate(zip(labels, values)):
        cx = x + 1.5 + i * cell_w
        ax.add_patch(Rectangle((cx, cell_y - 6.7), cell_w - 0.45, 6.6,
                               facecolor='white', edgecolor='#D5B4B4', linewidth=0.8, zorder=3))
        ax.text(cx + (cell_w - 0.45) / 2, cell_y - 1.6, label, ha='center', va='center',
                fontsize=7.2, color=COLORS['muted'], fontweight='bold', zorder=4)
        ax.text(cx + (cell_w - 0.45) / 2, cell_y - 4.7, value, ha='center', va='center',
                fontsize=11.0, color=COLORS['red'], fontweight='bold', zorder=4)
    ax.text(x + 1.5, y + 6.8, '条件：FHD · 5×5 · D=64 · PU=32 · 1 pixel/clock · 200 MHz',
            ha='left', va='center', fontsize=7.5, color=COLORS['ink'])
    ax.text(x + 1.5, y + 3.8, '性能估计：42 ms/frame（约 23.8 FPS）',
            ha='left', va='center', fontsize=7.5, color=COLORS['ink'])
    ax.text(x + 1.5, y + 1.2, '注意：文档未注明目标 FPGA/工具版本，也没有各子模块资源分摊。',
            ha='left', va='bottom', fontsize=7.1, color=COLORS['red'], fontweight='bold')


fig, ax = plt.subplots(figsize=(18, 10.2))
fig.patch.set_facecolor('white')
ax.set_xlim(0, 160)
ax.set_ylim(0, 92)
ax.set_aspect('equal')
ax.axis('off')

# Title and contract sentence.
ax.text(2, 89.5, 'Vitis Vision SGBM FPGA IP 核：电路结构与数据流',
        fontsize=19, fontweight='bold', color=COLORS['ink'], ha='left', va='top')
ax.text(2, 86.1,
        '源码配置：1920×1080 · XF_8UC1 · NPPC1 · Census 5×5 · D=64 · PU=32 · R=4 · P1/P2=20/40',
        fontsize=9.1, color=COLORS['muted'], ha='left', va='top')

# Kernel boundary.
kernel = FancyBboxPatch(
    (1.5, 39.2), 157, 42.7,
    boxstyle='round,pad=0.35,rounding_size=1.2', linewidth=1.5,
    edgecolor='#8897A5', facecolor='#FBFCFE', linestyle=(0, (5, 3)), zorder=0,
)
ax.add_patch(kernel)
ax.text(3.0, 80.4, 'sgbm_accel()  /  HLS DATAFLOW', fontsize=9.1,
        fontweight='bold', color=COLORS['muted'], ha='left', va='center')

# Control plane.
rounded_box(ax, 68.8, 78.0, 40.0, 3.0, 'AXI4-Lite 控制：rows · cols · penalty_small(P1) · penalty_large(P2)', (),
            edge=COLORS['muted'], face=COLORS['gray_fill'], title_size=7.3,
            align='center', title_y_pad=0.85, lw=1.0, radius=0.55)
control_arrow(ax, (88.8, 77.9), (88.8, 73.2), 'P1 / P2 / 尺寸')

# External memory and array conversion.
rounded_box(ax, 3.0, 47.2, 12.0, 26.2, '外部内存 / DDR',
            ('gmem0：左图', 'gmem1：右图', '', 'XF_8UC1', 'FHD 上限'),
            edge=COLORS['ink'], face=COLORS['gray_fill'], title_size=9.5,
            line_size=7.8, line_gap=3.0)

rounded_box(ax, 18.2, 62.4, 11.1, 9.8, 'Array2xfMat',
            ('左输入', '32-bit → 8-bit'), edge=COLORS['blue'], face=COLORS['blue_fill'],
            title_size=8.3, line_size=7.1, line_gap=2.45)
rounded_box(ax, 18.2, 48.2, 11.1, 9.8, 'Array2xfMat',
            ('右输入', '32-bit → 8-bit'), edge=COLORS['blue'], face=COLORS['blue_fill'],
            title_size=8.3, line_size=7.1, line_gap=2.45)

rounded_box(ax, 32.4, 62.0, 15.0, 10.8, '左 Census 5×5',
            ('24 个邻域比较', '5×1920×8-bit BRAM'),
            edge=COLORS['teal'], face=COLORS['teal_fill'], title_size=8.8,
            line_size=7.1, line_gap=2.55)
rounded_box(ax, 32.4, 47.8, 15.0, 10.8, '右 Census 5×5',
            ('24 个邻域比较', '5×1920×8-bit BRAM'),
            edge=COLORS['teal'], face=COLORS['teal_fill'], title_size=8.8,
            line_size=7.1, line_gap=2.55)

rounded_box(ax, 51.0, 51.0, 16.4, 18.4, 'Hamming 初始代价',
            ('xFSGBMcomputecost()', '', 'r_buff[64]×24-bit', '完全分区移位寄存器', '', '32 路 cost；2 批/像素', '输出：32×8-bit'),
            edge=COLORS['blue'], face=COLORS['blue_fill'], title_size=9.3,
            line_size=7.2, line_gap=2.35)

# Main data arrows into cost.
data_arrow(ax, (15.0, 67.2), (18.2, 67.2))
data_arrow(ax, (15.0, 53.0), (18.2, 53.0))
data_arrow(ax, (29.3, 67.2), (32.4, 67.2), '8-bit pixel', label_offset=(0, 1.05))
data_arrow(ax, (29.3, 53.0), (32.4, 53.0), '8-bit pixel', label_offset=(0, 1.05))
data_arrow(ax, (47.4, 67.4), (51.0, 64.4), '24-bit Census', connection='arc3,rad=-0.12',
           label_offset=(0.0, 1.5))
data_arrow(ax, (47.4, 53.2), (51.0, 56.1), '24-bit Census', connection='arc3,rad=0.12',
           label_offset=(0.0, -2.15))

# Optimization outer block and internals.
rounded_box(ax, 71.3, 43.8, 41.4, 32.7, '四路径 SGM 代价递推与聚合  ·  xFSGBMoptimization()', (),
            edge=COLORS['violet'], face='#F8F6FC', title_size=10.0,
            line_size=7.2, lw=1.8)
rounded_box(ax, 73.6, 58.4, 17.3, 12.8, '递推处理阵列',
            ('PU=32 路完全展开', 'R=4 路完全展开', '32×4 个逻辑递推 lane', '', 'disp_loop：2 批，II=2'),
            edge=COLORS['violet'], face=COLORS['violet_fill'], title_size=8.7,
            line_size=6.9, line_gap=2.08)
draw_path_icon(ax, 93.0, 59.0)
rounded_box(ax, 93.2, 50.9, 16.8, 6.5, '路径求和 ΣR',
            ('32×16-bit agg_cost',), edge=COLORS['violet'], face=COLORS['violet_fill'],
            title_size=8.1, line_size=6.8, line_gap=2.2, align='center')
rounded_box(ax, 73.6, 45.6, 36.4, 9.4, '路径历史状态（反馈存储）',
            ('Lr[3][64][1920]：360 KiB，路径/视差维完全分区，绑定 BRAM',
             'Lr_min[3][1920]：5.625 KiB；两者原始数据合计 365.625 KiB'),
            edge=COLORS['orange'], face=COLORS['orange_fill'], title_size=8.5,
            line_size=6.55, line_gap=2.38)

data_arrow(ax, (67.4, 60.2), (71.3, 60.2), '32×8-bit cost', label_offset=(0, 1.15))
data_arrow(ax, (90.9, 64.8), (93.2, 54.4), color=COLORS['violet'], lw=1.5,
           connection='arc3,rad=0.15')
data_arrow(ax, (110.0, 54.2), (115.9, 60.2), '32×16-bit', color=COLORS['violet'],
           connection='arc3,rad=-0.10', label_offset=(0, 1.3))
data_arrow(ax, (91.2, 49.9), (83.0, 58.4), 'Lr / min 反馈', color=COLORS['orange'], lw=1.65,
           connection='arc3,rad=0.23', label_offset=(0, -0.2))
data_arrow(ax, (84.4, 58.4), (93.5, 49.9), color=COLORS['orange'], lw=1.35,
           connection='arc3,rad=0.18')

# WTA and output.
rounded_box(ax, 115.9, 52.2, 14.0, 16.0, 'WTA 视差选择',
            ('xfSGBMcompute-', 'disparity()', '', '32 输入最小值树', '2 批比较', '', '输出：8-bit d'),
            edge=COLORS['green'], face=COLORS['green_fill'], title_size=8.8,
            line_size=6.9, line_gap=2.05)
rounded_box(ax, 133.0, 54.4, 10.4, 11.6, 'xfMat2Array',
            ('8-bit → 32-bit', '输出打包'), edge=COLORS['green'], face=COLORS['green_fill'],
            title_size=8.2, line_size=7.0, line_gap=2.4)
rounded_box(ax, 146.5, 52.2, 10.5, 16.0, '外部内存',
            ('gmem2', '', '视差图', 'XF_8UC1', '整数视差 0…63'),
            edge=COLORS['ink'], face=COLORS['gray_fill'], title_size=9.1,
            line_size=7.1, line_gap=2.35)
data_arrow(ax, (129.9, 60.2), (133.0, 60.2), '8-bit disparity', color=COLORS['green'],
           label_offset=(0, 1.2))
data_arrow(ax, (143.4, 60.2), (146.5, 60.2), color=COLORS['green'])

# Stage rate labels.
ax.text(59.2, 46.5, '目标：2 cycle/pixel（D/PU=2，II=1）', ha='center', va='center',
        fontsize=7.2, color=COLORS['blue'], fontweight='bold')
ax.text(92.0, 41.6, '默认稳态瓶颈：约 4 cycle/pixel（2 批 × II=2）', ha='center', va='center',
        fontsize=7.5, color=COLORS['violet'], fontweight='bold')
ax.text(122.9, 49.5, '目标：2 cycle/pixel', ha='center', va='center',
        fontsize=7.2, color=COLORS['green'], fontweight='bold')

# Bottom evidence panels.
panel_title(ax, 2.0, 36.1, 'a', '平台与接口口径')
rounded_box(ax, 2.0, 6.2, 46.8, 27.6, '默认设备 / 构建行为',
            ('L2 默认：PLATFORM=vck190，TARGET=hw_emu',
             'VCK190 分支请求 100 MHz；默认不是实体板运行',
             'L2 allowlist：vck190、u200',
             'U200 分支请求 300 MHz（请求值，不代表已收敛）',
             '',
             'AXI：gmem0 左图 · gmem1 右图 · gmem2 输出',
             '控制：AXI4-Lite；内部算法固定 NPPC1'),
            edge=COLORS['blue'], face=COLORS['blue_fill'], title_size=10.0,
            line_size=7.7, line_gap=3.0)

panel_title(ax, 54.0, 36.1, 'b', '源码可确定的结构规模')
rounded_box(ax, 54.0, 6.2, 49.7, 27.6, '结构规模（不是综合后的资源分摊）',
            ('• 双路 Census：2 × [5×1920×8-bit] BRAM 行缓存',
             '  加 2 × 5×5 局部窗口；每路形成 24-bit 描述子',
             '• Hamming：64×24-bit 右图历史；32 路并行，2 批/像素',
             '• SGM：32 PU × 4 path 完全展开；每候选执行 min4/惩罚/累加',
             '• 路径 BRAM 原始状态：365.625 KiB（Lr + Lr_min）',
             '• WTA：32 输入最小树，遍历两批得到 8-bit disparity',
             '',
             'HLS 可能共享、重定时或映射逻辑；结构数 ≠ LUT/DSP 实例数。'),
            edge=COLORS['teal'], face=COLORS['teal_fill'], title_size=10.0,
            line_size=7.25, line_gap=2.78)

panel_title(ax, 108.8, 36.1, 'c', '仓库给出的总核估算')
draw_resource_table(ax, 108.8, 6.2, 49.2, 27.6)

# Legend and provenance.
data_arrow(ax, (3.0, 3.5), (8.0, 3.5), color=COLORS['blue'], lw=1.9)
ax.text(8.7, 3.5, '像素 / 代价主数据流', fontsize=7.0, color=COLORS['muted'], va='center')
data_arrow(ax, (28.5, 3.5), (33.5, 3.5), color=COLORS['orange'], lw=1.6)
ax.text(34.2, 3.5, '路径历史反馈', fontsize=7.0, color=COLORS['muted'], va='center')
control_arrow(ax, (49.5, 3.5), (54.5, 3.5))
ax.text(55.2, 3.5, '控制流', fontsize=7.0, color=COLORS['muted'], va='center')
ax.text(158.0, 3.5,
        '依据：xf_sgbm_accel.cpp · xf_sgbm.hpp · xf_config_params.h · api-reference.rst',
        fontsize=6.8, color=COLORS['muted'], ha='right', va='center')

fig.subplots_adjust(left=0.012, right=0.988, top=0.985, bottom=0.02)
fig.savefig(f'{OUT_STEM}.svg', bbox_inches='tight', facecolor='white')
fig.savefig(f'{OUT_STEM}.pdf', bbox_inches='tight', facecolor='white')
fig.savefig(f'{OUT_STEM}.png', dpi=220, bbox_inches='tight', facecolor='white')

print(f'Wrote {OUT_STEM}.svg')
print(f'Wrote {OUT_STEM}.pdf')
print(f'Wrote {OUT_STEM}.png')
