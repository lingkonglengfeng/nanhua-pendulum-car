"""Build the compact V2 system relationship diagram.

This is a functional wiring relationship view, not an EDA schematic.  The
mapping comes from the checked-in V2 CSV and the accompanying design note.
The script intentionally does not import or execute draw_wiring.py because
that script writes the older diagrams.
"""
from __future__ import annotations

import csv
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.sax.saxutils import escape

import resvg_py


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"
CSV_PATH = OUT / "pin_assignment_v2.csv"
MD_PATH = OUT / "DMG474引脚与转接板方案_V2.md"
SVG_PATH = OUT / "03_原理图关系图_V2.svg"
PNG_PATH = OUT / "03_原理图关系图_V2.png"

W, H = 2400, 1500
FONT_FILE = "C:/Windows/Fonts/msyh.ttc"
COLORS = {
    "ink": "#203040",
    "muted": "#5d6d7b",
    "signal": "#2168a5",
    "encoder": "#7a4ba0",
    "low": "#277d58",
    "power": "#c3443b",
    "ground": "#5a6570",
    "pending": "#a16117",
    "panel": "#f5f8fb",
    "panel2": "#f9fbfd",
    "line": "#bfd0dc",
}


class SVG:
    def __init__(self, width: int, height: int, title: str):
        self.width = width
        self.height = height
        self.s: list[str] = [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            f"<title>{escape(title)}</title>",
            "<defs>",
            '<marker id="arrow-signal" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0 L10,5 L0,10 Z" fill="#2168a5"/></marker>',
            '<marker id="arrow-power" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0 L10,5 L0,10 Z" fill="#c3443b"/></marker>',
            '<marker id="arrow-low" markerWidth="10" markerHeight="10" refX="8" refY="5" orient="auto"><path d="M0,0 L10,5 L0,10 Z" fill="#277d58"/></marker>',
            '<filter id="shadow" x="-10%" y="-10%" width="120%" height="130%"><feDropShadow dx="0" dy="3" stdDeviation="4" flood-color="#193044" flood-opacity="0.12"/></filter>',
            "</defs>",
            '<style>text{font-family:"Microsoft YaHei","Noto Sans CJK SC",sans-serif} path,line,polyline{stroke-linecap:round;stroke-linejoin:round}</style>',
            '<rect width="2400" height="1500" fill="#ffffff"/>',
        ]

    def text(self, x: float, y: float, value: str, size: float = 18, color: str = "ink", anchor: str = "start", bold: bool = False) -> None:
        fill = COLORS.get(color, color)
        weight = 650 if bold else 400
        self.s.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{fill}" text-anchor="{anchor}" font-weight="{weight}">{escape(value)}</text>')

    def rect(self, x: float, y: float, w: float, h: float, fill: str = "#ffffff", stroke: str = "line", r: float = 12, dash: bool = False, shadow: bool = False) -> None:
        f = COLORS.get(fill, fill)
        st = COLORS.get(stroke, stroke)
        extra = ' stroke-dasharray="9 7"' if dash else ""
        extra += ' filter="url(#shadow)"' if shadow else ""
        self.s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{f}" stroke="{st}" stroke-width="2"{extra}/>')

    def line(self, x1: float, y1: float, x2: float, y2: float, color: str = "signal", width: float = 3, arrow: bool = False, dash: bool = False) -> None:
        st = COLORS.get(color, color)
        end = f' marker-end="url(#arrow-{color})"' if arrow and color in ("signal", "power", "low") else ""
        ds = ' stroke-dasharray="8 6"' if dash else ""
        self.s.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{st}" stroke-width="{width}"{ds}{end}/>')

    def path(self, d: str, color: str = "signal", width: float = 3, arrow: bool = False, dash: bool = False) -> None:
        st = COLORS.get(color, color)
        end = f' marker-end="url(#arrow-{color})"' if arrow and color in ("signal", "power", "low") else ""
        ds = ' stroke-dasharray="8 6"' if dash else ""
        self.s.append(f'<path d="{d}" fill="none" stroke="{st}" stroke-width="{width}"{ds}{end}/>')

    def dot(self, x: float, y: float, color: str = "signal", r: float = 5) -> None:
        self.s.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{COLORS.get(color, color)}"/>')

    def pill(self, x: float, y: float, value: str, color: str = "signal", width: float | None = None) -> None:
        width = width or max(72, 13 * len(value) + 18)
        self.rect(x, y - 20, width, 30, "#ffffff", color, 14)
        self.text(x + width / 2, y + 2, value, 15, color, "middle", True)

    def save(self) -> None:
        self.s.append("</svg>")
        SVG_PATH.write_text("\n".join(self.s), encoding="utf-8")
        ET.parse(SVG_PATH)
        PNG_PATH.write_bytes(resvg_py.svg_to_bytes(svg_path=str(SVG_PATH), font_family="Microsoft YaHei", font_files=[FONT_FILE]))


def read_inputs() -> tuple[dict[str, dict[str, dict[str, str]]], str]:
    """Load the confirmed CSV and note, then organize signal rows by wheel."""
    note = MD_PATH.read_text(encoding="utf-8")
    rows = list(csv.DictReader(CSV_PATH.open(encoding="utf-8-sig", newline="")))
    connector_pins = json.loads((OUT / "connector_pinout_verified.json").read_text(encoding="utf-8"))["pins"]
    for signal in rows:
        connector = signal["主板接口"]
        if connector in connector_pins:
            assert connector_pins[connector][signal["接口针号"]] == signal["MCU引脚"], signal
    assert len(rows) == 20, f"Unexpected checked input row count: {len(rows)}"
    by_wheel: dict[str, dict[str, dict[str, str]]] = {}
    for row in rows:
        wheel = row["轮位/模块"]
        net = row["网络"]
        by_wheel.setdefault(wheel, {})[net] = row
    required = {
        "PC6", "PC7", "CN10", "CN2", "DRV VCC", "悬空", "P7", "CN14",
    }
    assert "I2C4" in note and "不承载电机回流" in note
    assert "VCC" in note and "悬空" in note and "GH1.25" in note
    assert "P7/CN14" in note and "PC6/PC7" in note
    assert all(k in note for k in ("CN8", "CN10", "CN2"))
    return by_wheel, note


def row(d: dict[str, dict[str, str]], name: str) -> dict[str, str]:
    return d[name]


WHEELS = [
    ("左前", "FL", "#e9f2fb"),
    ("右前", "FR", "#eef8f2"),
    ("左后", "RL", "#f8f0fb"),
    ("右后", "RR", "#fff6e8"),
]


def draw_mainboard(v: SVG, by_wheel: dict[str, dict[str, dict[str, str]]]) -> None:
    x, y, w, h = 55, 150, 450, 930
    v.rect(x, y, w, h, "panel", "#6f92ae", 16, shadow=True)
    v.text(x + 25, y + 42, "主板 U1 · ATK-DMG474", 25, bold=True)
    v.text(x + 25, y + 72, "原始低压接口（CN针号按原理图）", 16, "muted")
    v.pill(x + 25, y + 108, "CN8 · PM1", "signal", 110)
    v.text(x + 25, y + 159, "FL  PA8 / PB13     PWM / DIR     17 / 18", 15, "ink")
    v.text(x + 25, y + 184, "FL  PA6 / PA7      ENC A / B      1 / 3", 15, "encoder")
    v.text(x + 25, y + 209, "FR  PA9 / PB14     PWM / DIR     19 / 20", 15, "ink")
    v.text(x + 25, y + 234, "RL  PA10 / PB15    PWM / DIR     21 / 22", 15, "ink")
    v.text(x + 25, y + 259, "RL  PA0 / PA1      ENC A / B      2 / 4", 15, "encoder")
    v.pill(x + 25, y + 304, "CN10 · PM2", "signal", 125)
    v.text(x + 25, y + 355, "FR  PD12 / PD13    ENC A / B      1 / 3", 15, "encoder")
    v.text(x + 25, y + 380, "RR  PC8 / PC10     PWM / DIR     21 / 18", 15, "ink")
    v.text(x + 25, y + 405, "RR  PD3 / PD4      ENC A / B      2 / 4", 15, "encoder")
    v.text(x + 25, y + 440, "I²C4  PC6 / PC7     SCL / SDA    17 / 19", 15, "signal")
    v.pill(x + 25, y + 488, "CN2 · 低压电源", "low", 145)
    v.text(x + 25, y + 538, "CN2-1 = 3.3V     传感器供电", 16, "low")
    v.text(x + 25, y + 565, "CN2-2 = GND      公共参考", 16, "ground")
    v.rect(x + 20, y + 610, w - 40, 190, "#ffffff", "#9db5c4", 10)
    v.text(x + 38, y + 645, "接入边界", 18, "ink", bold=True)
    for i, msg in enumerate([
        "CN8 / CN10 → J_MCU1 / J_MCU2",
        "接口触点 1:1；未使用触点不短接",
        "信号电平按 3.3V GPIO 处理",
        "I²C4 可并接 AS5600 / 预留 IMU",
    ]):
        v.text(x + 38, y + 680 + i * 29, msg, 15, "muted")
    v.text(x + 25, y + 852, "CN2 传感器供电兼容性待确认", 16, "pending")
    v.text(x + 25, y + 882, "（按外部负载预算核算，暂不视为已验证）", 14, "pending")


def draw_signal_board(v: SVG, by_wheel: dict[str, dict[str, dict[str, str]]]) -> None:
    x, y, w, h = 555, 150, 450, 930
    v.rect(x, y, w, h, "#f4faf7", "#70a68e", 16, shadow=True)
    v.text(x + 25, y + 42, "低压信号转接板", 25, bold=True)
    centers = [250, 475, 700, 925]
    for (wheel, tag, _), cy in zip(WHEELS, centers):
        v.rect(x + 24, cy - 32, w - 48, 72, "#ffffff", "#b8c9d3", 8)
        v.text(x + 38, cy - 6, f"J_DRV_{tag}  → PWM / DIR / GND", 16, "signal", bold=True)
        v.text(x + 38, cy + 23, f"J_ENC_{tag}  ← A / B；供电 3V3 / GND", 15, "encoder")
    v.text(x + 30, 345, "CN8 → J_MCU1：2×12，1:1", 17, "signal")
    v.text(x + 30, 378, "CN10 → J_MCU2：2×12，1:1", 17, "signal")
    v.text(x + 30, 410, "防呆接口；电机电流不经过此板", 16, "pending")
    v.text(x + 30, 560, "CN2-1 → 3V3_SENS", 18, "low")
    v.text(x + 30, 594, "CN2-2 → GND 公共参考", 18, "ground")
    v.text(x + 30, 628, "供编码器 / AS5600 / 预留 IMU", 17, "low")
    v.text(x + 30, 790, "I²C4 → AS5600 + 预留 IMU", 18, "signal")
    v.text(x + 30, 824, "SCL：PC6 / CN10-17", 17, "signal")
    v.text(x + 30, 856, "SDA：PC7 / CN10-19", 17, "signal")
    v.text(x + 30, 1010, "传感器供电兼容性、插头针序待确认", 15, "pending")
    v.text(x + 30, 1040, "I²C 上拉至 3.3V；地址与已有上拉先核对", 15, "pending")
    v.line(505, 360, 551, 360, "signal", 3, arrow=True)
    v.line(551, 390, 505, 390, "low", 2, arrow=True)
    v.line(505, 590, 551, 590, "low", 3, arrow=True)



def draw_wheel_card(v: SVG, wheel: str, tag: str, fill: str, d: dict[str, dict[str, str]], index: int, y: int) -> None:
    x, w, h = 1050, 1290, 205
    pwm = row(d, f"{tag}_PWM")
    direc = row(d, f"{tag}_DIR")
    enc_a = row(d, f"ENC_{tag}_A")
    enc_b = row(d, f"ENC_{tag}_B")
    timer_a = enc_a["外设"].replace("_CH1", "")
    timer_b = enc_b["外设"].replace("_CH2", "")
    v.rect(x, y, w, h, fill, "#9fb3c1", 15, shadow=True)
    v.text(x + 24, y + 34, f"{index}  {wheel}（{tag}）", 23, bold=True)
    v.text(x + 360, y + 34, f"J_DRV_{tag} + J_ENC_{tag} → DRV8871 + 编码器", 16, "muted")
    v.text(x + w - 24, y + 34, "DRV VCC 排针：悬空", 15, "pending", "end", True)
    v.line(x + 22, y + 51, x + w - 22, y + 51, "#bfd0dc", 1)
    left = x + 26
    right = x + 685
    v.text(left, y + 82, "DRV8871 低压控制", 17, "signal", bold=True)
    v.text(left, y + 112, f"IN1 / PWM   {pwm['MCU引脚']} · {pwm['外设']} · {pwm['主板接口']}-{pwm['接口针号']}", 16, "signal")
    v.text(left, y + 140, f"IN2 / DIR   {direc['MCU引脚']} · {direc['外设']} · {direc['主板接口']}-{direc['接口针号']}", 16, "signal")
    v.text(left, y + 172, "→ J_DRV：IN1 / IN2 / GND（只走低压信号）", 14, "muted")
    v.text(right, y + 82, f"编码器反馈 → MCU · {timer_a} 编码器模式", 17, "encoder", bold=True)
    v.text(right, y + 112, f"A           {enc_a['MCU引脚']} · {enc_a['外设']} · {enc_a['主板接口']}-{enc_a['接口针号']}", 16, "encoder")
    v.text(right, y + 140, f"B           {enc_b['MCU引脚']} · {enc_b['外设']} · {enc_b['主板接口']}-{enc_b['接口针号']}", 16, "encoder")
    v.text(right, y + 172, "→ J_ENC：3V3 / GND / A / B；供电电平待确认", 14, "muted")
    v.text(x + 24, y + 195, f"POWER +/− ← 独立分电；OUT1 / OUT2 → M{index} 电机绕组（粗线，不经信号板）", 14, "power", bold=True)


def draw_power(v: SVG) -> None:
    x, y, w, h = 55, 1135, 2285, 310
    v.rect(x, y, w, h, "#fff9f7", "#d59f98", 16, shadow=True)
    v.text(x + 25, y + 36, "电源与回流：功率链路独立于低压信号板", 23, "power", bold=True)
    v.text(x + w - 25, y + 36, "电机电流不走 CN8/CN10 或转接板", 16, "pending", "end", True)
    boxes = [(80, "B1 电池", "11.1V 标称"), (330, "F1 保险", "按峰值/线材选"), (505, "S0 总开关", "主板与电机总断"), (740, "VBAT_SW 分支", "总开关后电源节点"), (975, "S1 电机开关", "仅断电机侧"), (1170, "独立分电", "粗线 / 独立回流")]
    for bx, title, sub in boxes:
        v.rect(x + bx, y + 72, 155 if bx not in (740, 1170) else 180, 66, "#ffffff", "#d7a29b", 9)
        v.text(x + bx + (77 if bx not in (740, 1170) else 90), y + 99, title, 16, "power", "middle", True)
        v.text(x + bx + (77 if bx not in (740, 1170) else 90), y + 122, sub, 12, "muted", "middle")
    for a, b in [(235, 330), (485, 505), (660, 740), (920, 975), (1130, 1170)]:
        v.line(x + a, y + 105, x + b, y + 105, "power", 4, arrow=True)
    v.line(x + 830, y + 138, x + 830, y + 174, "power", 3, arrow=True)
    v.text(x + 842, y + 170, "VBAT_SW → 主板 DC 输入", 14, "power")
    # Four independent driver power branches.
    for i, (tag, bx) in enumerate(zip(("FL", "FR", "RL", "RR"), (1375, 1585, 1795, 2005))):
        v.line(x + 1260, y + 138, x + bx + 48, y + 170, "power", 2, arrow=True)
        v.rect(x + bx, y + 180, 160, 53, "#ffffff", "#d7a29b", 8)
        v.text(x + bx + 80, y + 202, f"DRV_{tag} POWER +", 14, "power", "middle", True)
        v.text(x + bx + 80, y + 222, "POWER− 接下方公共地", 12, "muted", "middle")
    v.line(x + 80, y + 138, x + 80, y + 265, "ground", 3)
    for bx in (1375, 1585, 1795, 2005):
        v.line(x + bx + 80, y + 233, x + bx + 80, y + 265, "ground", 3)
        v.dot(x + bx + 80, y + 265, "ground", 3)
    # Common reference and separate high-current return.
    v.line(x + 80, y + 265, x + 1260, y + 265, "ground", 3)
    v.dot(x + 80, y + 265, "ground")
    v.text(x + 95, y + 260, "GND 公共参考：主板 / 信号板 / DRV 逻辑", 14, "ground")
    v.line(x + 1260, y + 265, x + 2175, y + 265, "ground", 5, dash=True)
    v.text(x + 1440, y + 291, "电机 POWER− 粗线独立回分电点（不穿过信号板）", 14, "ground")


def draw_notes(v: SVG) -> None:
    y = 1465
    v.text(55, y, "核对项：模块插头物理针序待核对（尤其 AS5600 GH1.25）；PC6/PC7 共享辅助电路，断开 P7/CN14 步进供电并核对波形后再启用 I²C4。", 15, "pending")
    v.text(2345, y, "关系图 · 非 Gerber · 未完成 ERC/DRC", 15, "pending", "end", True)


def main() -> None:
    by_wheel, _note = read_inputs()
    # Keep the mapping visibly anchored to the input rows and catch accidental edits.
    expected = {
        "左前": ("PA8", "PB13", "PA6", "PA7"),
        "右前": ("PA9", "PB14", "PD12", "PD13"),
        "左后": ("PA10", "PB15", "PA0", "PA1"),
        "右后": ("PC8", "PC10", "PD3", "PD4"),
    }
    for wheel, tag, _ in WHEELS:
        got = tuple(row(by_wheel[wheel], net)["MCU引脚"] for net in (f"{tag}_PWM", f"{tag}_DIR", f"ENC_{tag}_A", f"ENC_{tag}_B"))
        assert got == expected[wheel], (wheel, got)

    v = SVG(W, H, "四驱倒立摆 V2 原理图关系图 · DMG474 + 四路 DRV8871")
    v.text(55, 58, "四驱倒立摆 · 原理图关系图 V2（座子脚号已勘误）", 34, "ink", bold=True)
    v.text(55, 94, "DMG474 主板 → 低压信号转接板 → 四路 DRV8871 / 编码器；按已核对 CSV 展示确切 GPIO、CN 针号与定时器。", 18, "muted")
    v.text(2345, 58, "03 / V2", 17, "muted", "end")
    # Lane labels make the long horizontal signal arrows self-explanatory.
    v.text(510, 120, "主板接口", 15, "signal", "end", True)
    v.text(1008, 120, "低压分组信号", 15, "low", "end", True)
    draw_mainboard(v, by_wheel)
    draw_signal_board(v, by_wheel)
    for i, (wheel, tag, fill) in enumerate(WHEELS, 1):
        cy = 250 + (i - 1) * 225
        # The board and each card have a dedicated horizontal lane; no signal lines cross.
        v.line(1005, cy - 15, 1048, cy - 15, "signal", 2, arrow=True)
        v.line(1048, cy + 23, 1005, cy + 23, "low", 2, arrow=True)
        draw_wheel_card(v, wheel, tag, fill, by_wheel[wheel], i, 150 + (i - 1) * 225)
    draw_power(v)
    draw_notes(v)
    v.save()
    print(f"Wrote and parsed {SVG_PATH}")
    print(f"Rendered {PNG_PATH} ({PNG_PATH.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
