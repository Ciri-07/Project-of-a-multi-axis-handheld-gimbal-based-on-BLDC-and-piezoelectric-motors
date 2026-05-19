from pathlib import Path
import textwrap

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "线性霍尔传感器" / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)


def font(size, bold=False):
    candidates = [
        Path(r"C:\Windows\Fonts\msyhbd.ttc") if bold else Path(r"C:\Windows\Fonts\msyh.ttc"),
        Path(r"C:\Windows\Fonts\simhei.ttf"),
        Path(r"C:\Windows\Fonts\simsun.ttc"),
    ]
    for p in candidates:
        if p.exists():
            return ImageFont.truetype(str(p), size)
    return ImageFont.load_default()


FONT_TITLE = font(36, True)
FONT_BOX = font(24)
FONT_SMALL = font(20)


COLORS = {
    "bg": "#F7FAFC",
    "title": "#1F2937",
    "box": "#FFFFFF",
    "box2": "#EAF4FF",
    "border": "#2563EB",
    "border2": "#0F766E",
    "arrow": "#475569",
    "lane": "#EEF6FF",
    "lane2": "#F0FDF4",
    "lane3": "#FFF7ED",
    "lane4": "#F5F3FF",
}


def wrap_cn(text, max_len):
    parts = []
    for raw in text.split("\n"):
        line = ""
        count = 0
        for ch in raw:
            count += 1.8 if ord(ch) > 127 else 1
            line += ch
            if count >= max_len:
                parts.append(line)
                line = ""
                count = 0
        if line:
            parts.append(line)
    return parts or [""]


def center_text(draw, box, text, fnt, fill="#111827", max_len=12, line_gap=6):
    x1, y1, x2, y2 = box
    lines = wrap_cn(text, max_len)
    heights = []
    widths = []
    for line in lines:
        bb = draw.textbbox((0, 0), line, font=fnt)
        widths.append(bb[2] - bb[0])
        heights.append(bb[3] - bb[1])
    total_h = sum(heights) + line_gap * (len(lines) - 1)
    y = y1 + (y2 - y1 - total_h) / 2 - 2
    for line, w, h in zip(lines, widths, heights):
        x = x1 + (x2 - x1 - w) / 2
        draw.text((x, y), line, font=fnt, fill=fill)
        y += h + line_gap


def arrow(draw, start, end, color=COLORS["arrow"], width=5):
    sx, sy = start
    ex, ey = end
    draw.line((sx, sy, ex, ey), fill=color, width=width)
    if abs(ex - sx) >= abs(ey - sy):
        sign = 1 if ex >= sx else -1
        pts = [(ex, ey), (ex - sign * 18, ey - 10), (ex - sign * 18, ey + 10)]
    else:
        sign = 1 if ey >= sy else -1
        pts = [(ex, ey), (ex - 10, ey - sign * 18), (ex + 10, ey - sign * 18)]
    draw.polygon(pts, fill=color)


def draw_box(draw, box, text, fill=COLORS["box"], border=COLORS["border"], max_len=12):
    draw.rounded_rectangle(box, radius=20, fill=fill, outline=border, width=3)
    center_text(draw, box, text, FONT_BOX, max_len=max_len)


def save_horizontal(name, title, steps, subtitle=None):
    w = 2100
    h = 520 if len(steps) <= 8 else 720
    img = Image.new("RGB", (w, h), COLORS["bg"])
    draw = ImageDraw.Draw(img)
    draw.text((60, 35), title, font=FONT_TITLE, fill=COLORS["title"])
    if subtitle:
        draw.text((62, 88), subtitle, font=FONT_SMALL, fill="#64748B")

    if len(steps) <= 8:
        y = 220
        box_w, box_h, gap = 210, 120, 42
        x = 70
        for idx, step in enumerate(steps):
            box = (x, y, x + box_w, y + box_h)
            draw_box(draw, box, step, fill=COLORS["box2"] if idx % 2 else COLORS["box"], max_len=10)
            if idx < len(steps) - 1:
                arrow(draw, (x + box_w + 8, y + box_h / 2), (x + box_w + gap - 8, y + box_h / 2))
            x += box_w + gap
    else:
        cols = 5
        box_w, box_h = 300, 110
        gap_x, gap_y = 82, 90
        x0, y0 = 120, 190
        positions = []
        for i, step in enumerate(steps):
            row = i // cols
            col = i % cols
            if row % 2 == 0:
                x = x0 + col * (box_w + gap_x)
            else:
                x = x0 + (cols - 1 - col) * (box_w + gap_x)
            y = y0 + row * (box_h + gap_y)
            positions.append((x, y, x + box_w, y + box_h))
            draw_box(draw, positions[-1], step, fill=COLORS["box2"] if i % 2 else COLORS["box"], max_len=14)
        for i in range(len(positions) - 1):
            b1, b2 = positions[i], positions[i + 1]
            c1y = (b1[1] + b1[3]) / 2
            c2y = (b2[1] + b2[3]) / 2
            if abs(c2y - c1y) > 40:
                start = ((b1[0] + b1[2]) / 2, b1[3] + 8)
                end = ((b2[0] + b2[2]) / 2, b2[1] - 8)
            elif b2[0] > b1[0]:
                start = (b1[2] + 8, c1y)
                end = (b2[0] - 8, c2y)
            else:
                start = (b1[0] - 8, c1y)
                end = (b2[2] + 8, c2y)
            arrow(draw, start, end)

    path = FIG_DIR / name
    img.save(path, quality=95)
    return path


def save_overall():
    w, h = 2200, 980
    img = Image.new("RGB", (w, h), COLORS["bg"])
    draw = ImageDraw.Draw(img)
    draw.text((60, 35), "线性 Hall + IMU 高精度检测总体融合框图", font=FONT_TITLE, fill=COLORS["title"])
    draw.text((62, 88), "结构可测、实时解算、个体标定、融合控制四层协同", font=FONT_SMALL, fill="#64748B")

    lanes = [
        ("结构层：保证可测", COLORS["lane"], ["磁钢/气隙/Hall位置设计", "稳定正余弦磁场"]),
        ("信号层：实时解算", COLORS["lane2"], ["Hall原始电压", "零偏/幅值/相位校正", "atan2解角", "谐波补偿+ESO"]),
        ("标定层：个体补偿", COLORS["lane3"], ["低速扫角数据", "周期误差辨识", "误差表/傅里叶参数"]),
        ("融合控制层", COLORS["lane4"], ["IMU角速度/姿态", "IMU-Hall融合", "位置/速度/FOC控制"]),
    ]
    y = 160
    lane_h = 170
    for li, (label, fill, nodes) in enumerate(lanes):
        draw.rounded_rectangle((70, y, w - 70, y + lane_h), radius=24, fill=fill, outline="#CBD5E1", width=2)
        draw.text((100, y + 22), label, font=font(26, True), fill="#0F172A")
        x = 430
        bw = 300 if len(nodes) >= 4 else 360
        gap = 70
        boxes = []
        for idx, n in enumerate(nodes):
            b = (x, y + 45, x + bw, y + 130)
            boxes.append(b)
            draw_box(draw, b, n, fill="#FFFFFF", border=COLORS["border2"] if li in (0, 2) else COLORS["border"], max_len=13)
            x += bw + gap
        for b1, b2 in zip(boxes, boxes[1:]):
            arrow(draw, (b1[2] + 8, (b1[1] + b1[3]) / 2), (b2[0] - 8, (b2[1] + b2[3]) / 2))
        y += lane_h + 35

    arrow(draw, (840, 290), (540, 375), color="#2563EB")
    arrow(draw, (1510, 705), (1360, 800), color="#F97316")
    arrow(draw, (1550, 500), (1150, 800), color="#16A34A")

    path = FIG_DIR / "linear_hall_overall_fusion.png"
    img.save(path, quality=95)
    return path


def main():
    save_horizontal(
        "linear_hall_method1_self_calibration.png",
        "方法一：闭环自校准与数据驱动补偿",
        ["Hall原始电压", "零偏/幅值预处理", "Clarke/正余弦构造", "atan2初始角度", "闭环采集数据", "非线性模型辨识", "补偿表/函数", "高精度轴角"],
        "目标：不用昂贵外部编码器，利用闭环数据识别并补偿周期误差",
    )
    save_horizontal(
        "linear_hall_method2_sem_eso.png",
        "方法二：归一化 + 谐波重构 + ESO 复合信号提取",
        ["两路Hall信号", "零偏校正", "幅值归一化", "FFT识别谐波", "建立谐波模型", "多角公式重构", "抵消主要谐波", "ESO估计残差", "atan2输出角度", "轴角速度反馈"],
        "目标：抑制幅值不一致、高次谐波与未建模噪声，适合低速云台",
    )
    save_horizontal(
        "linear_hall_method3_structure.png",
        "方法三：传感器与电机结构一体化自位置检测",
        ["结构需求", "磁钢/磁路设计", "仿真Hall磁场", "确定Hall位置/气隙", "PCB与机械集成", "采集一圈数据", "零偏/增益/相位标定", "atan2/查表解角", "闭环位置反馈", "结构迭代优化"],
        "目标：在结构设计阶段提高原始信号质量，降低后续算法补偿压力",
    )
    save_overall()
    print(FIG_DIR)


if __name__ == "__main__":
    main()
