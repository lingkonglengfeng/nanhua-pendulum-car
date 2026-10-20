"""Build a compact connector pinout reference drawing.

This file deliberately contains the reviewed CN8/CN10 mapping inline.  It is a
schematic reference (two 2x12 connector symbols), rather than a PCB or mating
connector top view.  The output is kept deliberately local: labels are the
connection points and no long harness wires are drawn.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from xml.sax.saxutils import escape

import resvg_py


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"
SVG_PATH = OUT / "04_双排座原理图参考.svg"
PNG_PATH = OUT / "04_双排座原理图参考.png"
FONT_FILE = "C:/Windows/Fonts/msyh.ttc"

W, H = 2600, 1980
INK = "#243443"
MUTED = "#657786"
BLUE = "#1b68a4"
PURPLE = "#75479a"
TEAL = "#157b7a"
RED = "#bd5146"
GREEN = "#2e7d59"
GROUND = "#56616c"
GRID = "#bccbd5"
PANEL = "#f6f9fb"
NC_FILL = "#f1f3f5"


class SVG:
    def __init__(self, width: int, height: int) -> None:
        self.s: list[str] = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<title>DMG474 CN8/CN10 双排座原理图参考</title>',
            '<defs><filter id="shadow" x="-10%" y="-10%" width="120%" height="130%"><feDropShadow dx="0" dy="3" stdDeviation="3" flood-color="#203040" flood-opacity="0.13"/></filter></defs>',
            '<style>text{font-family:"Microsoft YaHei","Noto Sans CJK SC",sans-serif} line,path,rect,circle,polyline{stroke-linecap:round;stroke-linejoin:round}</style>',
            f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        ]

    def text(self, x: float, y: float, value: str, size: float = 18, color: str = INK,
             anchor: str = "start", bold: bool = False) -> None:
        weight = 650 if bold else 400
        self.s.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color}" text-anchor="{anchor}" font-weight="{weight}">{escape(value)}</text>')

    def rect(self, x: float, y: float, w: float, h: float, fill: str = "#fff",
             stroke: str = GRID, r: float = 10, shadow: bool = False, dash: bool = False) -> None:
        extra = (' filter="url(#shadow)"' if shadow else '') + (' stroke-dasharray="9 7"' if dash else '')
        self.s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="2"{extra}/>')

    def line(self, x1: float, y1: float, x2: float, y2: float, color: str = GRID,
             width: float = 2, dash: bool = False) -> None:
        extra = ' stroke-dasharray="7 6"' if dash else ''
        self.s.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="{width}"{extra}/>')

    def circle(self, x: float, y: float, r: float, fill: str, stroke: str = INK, width: float = 2) -> None:
        self.s.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')

    def save(self) -> None:
        self.s.append("</svg>")
        OUT.mkdir(parents=True, exist_ok=True)
        SVG_PATH.write_text("\n".join(self.s), encoding="utf-8")
        ET.parse(SVG_PATH)
        PNG_PATH.write_bytes(resvg_py.svg_to_bytes(svg_path=str(SVG_PATH), font_family="Microsoft YaHei", font_files=[FONT_FILE]))


# (pin, MCU pad, net label).  Pin numbering is the schematic numbering: odd
# pins are on the left column and even pins on the right column, top to bottom.
CN8 = [
    (1, "PA6", "ENC_FL_A"), (2, "PA0", "ENC_RL_A"),
    (3, "PA7", "ENC_FL_B"), (4, "PA1", "ENC_RL_B"),
    (5, "PE4", "NC"), (6, "PA2", "NC"),
    (7, "GND", "GND"), (8, "PA3", "NC"),
    (9, "PC5", "NC"), (10, "PC1", "NC"),
    (11, "PB2", "NC"), (12, "PC2", "NC"),
    (13, "PC4", "NC"), (14, "PC3", "NC"),
    (15, "PC0", "NC"), (16, "GND", "GND"),
    (17, "PA8", "FL_PWM"), (18, "PB13", "FL_DIR"),
    (19, "PA9", "FR_PWM"), (20, "PB14", "FR_DIR"),
    (21, "PA10", "RL_PWM"), (22, "PB15", "RL_DIR"),
    (23, "5V", "NC"), (24, "PC13", "NC"),
]
CN10 = [
    (1, "PD12", "ENC_FR_A"), (2, "PD3", "ENC_RR_A"),
    (3, "PD13", "ENC_FR_B"), (4, "PD4", "ENC_RR_B"),
    (5, "PD14", "NC"), (6, "PD7", "NC"),
    (7, "GND", "GND"), (8, "PB1", "NC"),
    (9, "PB0", "NC"), (10, "PE7", "NC"),
    (11, "PA4", "NC"), (12, "PE8", "NC"),
    (13, "PA5", "NC"), (14, "PE10", "NC"),
    (15, "PE9", "NC"), (16, "GND", "GND"),
    (17, "PC6", "I2C_SCL"), (18, "PC10", "RR_DIR"),
    (19, "PC7", "I2C_SDA"), (20, "PC11", "NC"),
    (21, "PC8", "RR_PWM"), (22, "PC12", "NC"),
    (23, "5V", "NC"), (24, "PD11", "NC"),
]


def role_color(net: str) -> str:
    if net == "GND":
        return GROUND
    if net == "NC":
        return MUTED
    if net.startswith("ENC_"):
        return PURPLE
    if net.startswith("I2C"):
        return TEAL
    return BLUE


def draw_connector(v: SVG, x: int, y: int, title: str, rows: list[tuple[int, str, str]], note: str = "NC 请画不连接叉号，不能当作公共网名；23脚主板5V留空") -> None:
    pw, ph = 1190, 900
    v.rect(x, y, pw, ph, PANEL, "#7f9bad", 16, shadow=True)
    v.text(x + 28, y + 42, title, 27, INK, bold=True)
    v.text(x + pw - 28, y + 40, "2×12 原理图符号 · 1脚与防呆口按实物核对", 14, MUTED, "end")
    # Connector body and keyed top edge.  The columns are schematic pins,
    # intentionally not a top-view drawing of the mating plug.
    bx, by, bw, bh = x + 355, y + 80, 480, 755
    v.rect(bx, by, bw, bh, "#ffffff", "#547083", 9)
    v.line(bx + 16, by + 34, bx + bw - 16, by + 34, "#547083", 3)
    v.text(bx + bw / 2, by + 63, "原理图符号：奇数左 / 偶数右", 14, MUTED, "middle")
    x_left, x_right = bx + 70, bx + bw - 70
    top = by + 100
    step = 52
    for i in range(12):
        odd = rows[2 * i]
        even = rows[2 * i + 1]
        cy = top + i * step
        for xpos, (pin, pad, net), side in ((x_left, odd, "left"), (x_right, even, "right")):
            color = role_color(net)
            v.circle(xpos, cy, 12, "#ffffff", color, 3)
            v.text(xpos, cy + 5, str(pin), 14, color, "middle", True)
            if side == "left":
                label = f"{pin:02d}  {pad}  {net}"
                # Keep labels outside the connector body but inside its panel;
                # this prevents the two adjacent connector panels from
                # colliding in the centre gutter.
                v.text(x + 328, cy + 5, label, 16, color, "end", net != "NC")
                v.line(x + 338, cy, xpos - 12, cy, color, 2)
            else:
                label = f"{pin:02d}  {pad}  {net}"
                v.text(x + 862, cy + 5, label, 16, color, "start", net != "NC")
                v.line(xpos + 12, cy, x + 850, cy, color, 2)
    v.text(x + 28, y + 864, note, 14, MUTED)


def box(v: SVG, x: int, y: int, w: int, h: int, title: str, lines: list[tuple[str, str]], stroke: str = GRID) -> None:
    v.rect(x, y, w, h, "#fbfcfd", stroke, 12)
    v.text(x + 18, y + 31, title, 19, INK, bold=True)
    yy = y + 61
    for text, color in lines:
        v.text(x + 18, yy, text, 15, color)
        yy += 25


def draw_headers(v: SVG) -> None:
    y = 1110
    v.text(55, y - 20, "转接板端子模板（自定义线序；每个轮位复制一套）", 22, INK, bold=True)
    # Four identical DRV8871 low-voltage control headers.
    for i, tag in enumerate(("FL", "FR", "RL", "RR")):
        x = 55 + i * 630
        v.rect(x, y, 585, 180, "#f5faff", "#8caec8", 12)
        v.text(x + 18, y + 29, f"J_DRV_{tag}  · DRV8871", 18, BLUE, bold=True)
        v.text(x + 18, y + 57, "1  GND", 16, GROUND, True)
        v.text(x + 18, y + 84, f"2  IN1 / PWM   {tag}_PWM", 16, BLUE)
        v.text(x + 18, y + 111, f"3  IN2 / DIR   {tag}_DIR", 16, BLUE)
        v.text(x + 18, y + 143, "仅接 GND/IN1/IN2；模块 VM 针留空", 13, MUTED)
    # Four identical encoder headers and one AS5600 header.
    y2 = 1330
    v.text(55, y2 - 20, "反馈与 I²C 端子（自定义线序；不是现有模块的原厂针序）", 22, INK, bold=True)
    for i, tag in enumerate(("FL", "FR", "RL", "RR")):
        x = 55 + i * 475
        v.rect(x, y2, 435, 170, "#fbf7ff", "#b195c2", 12)
        v.text(x + 16, y2 + 28, f"J_ENC_{tag}", 18, PURPLE, bold=True)
        v.text(x + 16, y2 + 58, "1  3V3", 15, GREEN)
        v.text(x + 16, y2 + 84, "2  GND", 15, GROUND)
        v.text(x + 16, y2 + 110, f"3  A   ENC_{tag}_A", 15, PURPLE)
        v.text(x + 16, y2 + 136, f"4  B   ENC_{tag}_B", 15, PURPLE)
    x = 1980
    v.rect(x, y2, 565, 170, "#f4fbfb", "#83b7b5", 12)
    v.text(x + 18, y2 + 28, "J_AS5600  · I²C", 18, TEAL, bold=True)
    v.text(x + 18, y2 + 58, "1  3V3", 15, GREEN)
    v.text(x + 18, y2 + 84, "2  GND", 15, GROUND)
    v.text(x + 18, y2 + 110, "3  SCL   PC6 / I2C_SCL", 15, TEAL)
    v.text(x + 18, y2 + 136, "4  SDA   PC7 / I2C_SDA", 15, TEAL)


def draw_notes(v: SVG, y: int = 1570) -> None:
    v.rect(55, y, 2490, 335, "#fffaf4", "#d3ae78", 14)
    v.text(78, y + 35, "供电、核对与边界说明", 22, RED, bold=True)
    notes = [
        ("CN2-1 = 独立 3.3V → 3V3_SENS；CN2-2 = GND → 公共参考。主控 3.3V 不从 CN8/CN10 的 23脚 5V 引入。", RED),
        ("CN8-7/CN8-16、CN10-7/CN10-16 为 GND；四个驱动座 1脚及编码器/AS5600 2脚均接此公共 GND。", GROUND),
        ("PC6/PC7 还连接板载步进/舵机电路：核对并断开 P7/CN14 步进供电路径后再接 I²C；上拉至 3.3V，4.7k 可选，模块已有上拉先核对。", TEAL),
        ("图示为原理图符号与网络标签，不是排针实物俯视图；封装以实际 1脚和防呆口为准，排线逐针 1对1。", MUTED),
        ("电机功率线（VM、OUT1/OUT2）不经过 CN8/CN10 或这些低压转接座；DRV VM 排针留空，电机电源独立分电。", RED),
    ]
    yy = y + 76
    for text, color in notes:
        v.text(78, yy, "• " + text, 15, color)
        yy += 45
    v.text(78, y + 300, "蓝 = PWM/DIR　紫 = 编码器 A/B　青 = I²C　绿 = 3V3　灰 = GND　浅灰 = NC", 14, MUTED)


def main() -> None:
    v = SVG(W, H)
    v.text(55, 52, "DMG474 · CN8 / CN10 双排座原理图参考", 31, INK, bold=True)
    v.text(55, 88, "脚号已勘误 · 同名信号网标相连；传感器3.3V供电兼容性及模块端针序待确认", 16, MUTED)
    v.text(W - 55, 52, "REV · 2026-09-28", 15, MUTED, "end")
    draw_connector(v, 55, 135, "J1  ·  接主板 CN8 / PM1", CN8)
    draw_connector(v, 1355, 135, "J2  ·  接主板 CN10 / PM2", CN10)
    # A short center legend keeps the drawing explicit without routing long wires.
    v.rect(1137, 875, 326, 86, "#f6fbf7", "#8fb9a0", 10)
    v.text(1300, 906, "公共参考", 17, GREEN, "middle", True)
    v.text(1300, 934, "GND: 7/16脚 · 3V3: CN2-1", 14, GROUND, "middle")
    draw_headers(v)
    draw_notes(v)
    v.save()


if __name__ == "__main__":
    main()
