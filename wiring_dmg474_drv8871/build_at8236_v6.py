"""AT8236 two-module pin and ribbon-cable reference from purchased module photos."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import build_connector_reference as base
import build_two_cable_v4 as v4

OUT = Path(__file__).resolve().parent / "output"
NAME = "08_AT8236双模块双牛角座_V6"

MODULE_PINOUT = [
    ("L", 1, "AOUT2"), ("L", 2, "AOUT1"), ("L", 3, "BIN1"),
    ("L", 4, "BIN2"), ("L", 5, "VCC"), ("L", 6, "VM"), ("L", 7, "GND"),
    ("R", 1, "GND"), ("R", 2, "VM"), ("R", 3, "VCC"),
    ("R", 4, "AIN2"), ("R", 5, "AIN1"), ("R", 6, "BOUT1"), ("R", 7, "BOUT2"),
]


def assignment():
    with (OUT / "pin_assignment_v4_two_cables.csv").open(encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        rows = list(reader)
    assert fields is not None
    paired = {"FL": ("FRONT", "A"), "FR": ("FRONT", "B"),
              "RL": ("REAR", "A"), "RR": ("REAR", "B")}
    for row in rows:
        net = row["网络"]
        tag = net[:2]
        if tag not in paired:
            continue
        module, channel = paired[tag]
        if net == f"{tag}_PWM":
            row.update({"目标端": f"AT_{module}.{channel}IN1", "状态": "PWM输入1"})
        elif net == f"{tag}_DIR":
            row.update({"网络": f"{tag}_IN2", "目标端": f"AT_{module}.{channel}IN2",
                        "状态": "第二控制输入；不是模块DIR脚"})
    signals = [r for r in rows if r["MCU引脚"] != "无"]
    assert len(signals) == len({r["MCU引脚"] for r in signals}) == 23
    truth = json.loads((OUT / "connector_pinout_verified.json").read_text(encoding="utf-8"))["pins"]
    for row in signals:
        if row["主板接口"] in truth:
            assert truth[row["主板接口"]][row["接口针号"]] == row["MCU引脚"], row
    with (OUT / "pin_assignment_v6_at8236.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def module_pin_rows(module: str, tags: tuple[str, str]):
    a, b = tags
    nets = {
        "AOUT1": f"{a}_MOTOR1", "AOUT2": f"{a}_MOTOR2",
        "BOUT1": f"{b}_MOTOR1", "BOUT2": f"{b}_MOTOR2",
        "AIN1": f"{a}_PWM", "AIN2": f"{a}_IN2",
        "BIN1": f"{b}_PWM", "BIN2": f"{b}_IN2",
        "VCC": "3V3_LOCAL", "VM": "VM_MOTOR", "GND": "GND",
    }
    return [(module, side, pos, pin, nets[pin]) for side, pos, pin in MODULE_PINOUT]


def write_module_pinout():
    rows = module_pin_rows("AT_FRONT", ("FL", "FR")) + module_pin_rows("AT_REAR", ("RL", "RR"))
    with (OUT / "AT8236_module_headers_v6.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(("模块", "照片正面视角列", "从上到下序号", "模块丝印", "转接板网络"))
        writer.writerows(rows)


def draw_module(v, x, y, name, tags):
    a, b = tags
    v.rect(x, y, 1190, 480, "#f2f8fc", "#8caec8", 15)
    v.text(x + 25, y + 40, f"{name}  ·  A路={a} / B路={b}", 23, base.BLUE, bold=True)
    v.text(x + 25, y + 75, "按用户照片正面、文字正立时，从上到下；不是底视图", 17, base.RED)
    data = module_pin_rows(name, tags)
    for i in range(7):
        left = data[i]
        right = data[7+i]
        yy = y + 117 + i * 42
        v.text(x + 30, yy, f"L{i+1}  {left[3]}  →  {left[4]}", 17, base.BLUE if "IN" in left[3] else base.INK)
        v.text(x + 625, yy, f"R{i+1}  {right[3]}  →  {right[4]}", 17, base.BLUE if "IN" in right[3] else base.INK)
    v.text(x + 25, y + 445, "两排 7针 · 2.54 mm针距；排距、插座方向与脚位逐个核对实物", 16, base.RED)


def draw_notes(v):
    y = 1625
    v.rect(55, y, 2490, 215, "#f2fafc", "#7daeb9", 14)
    v.text(80, y + 40, "控制、电源和屏幕", 22, base.TEAL, bold=True)
    v.text(80, y + 80, "IN1←原 PWM   IN2←原方向 GPIO；10 正向，01 反向，00 滑行，11 刹车。反向占空比需取反。", 19, base.BLUE)
    v.text(80, y + 119, "模块 VM 两脚←3S 电机母线；VCC 两脚←3V3_LOCAL；GND 两脚←公共地。", 18, base.GREEN)
    v.text(80, y + 157, "IPS200 SPI3、四路编码器、AS5600 均沿用 V4；CN10-23 的 5V 只供本地稳压。", 18, base.MUTED)
    v.text(80, y + 194, "J_IPS 自定义 8 针：1 GND / 2 3V3 / 3 SCK / 4 MOSI / 5 CS / 6 DC / 7 RST / 8 BLK预留。", 17, base.TEAL)
    y = 1860
    v.rect(55, y, 2490, 315, "#fffaf4", "#d3ae78", 14)
    v.text(80, y + 41, "投板前必须实测/核对", 22, base.RED, bold=True)
    notes = [
        "AT8236 芯片是单路 H 桥；此双路模块有两颗芯片，四电机共需两块模块。",
        "照片所示模块标称 5–17V、每路 3A；12V 电机额定0.45A/堵转1.3A，仍不允许持续堵转或带载瞬间反转。",
        "电机功率 VM、AOUT/BOUT 经模块排针与转接板宽铜/端子；不能通过主板24芯信号排线供电。",
        "图给出两排针的丝印顺序；实际 1脚、排距、针脚方向、与电源输入的连通性按实物确认。",
        "稳压器 3.3V 给模块 VCC；模块 VM 独立接 3S。V5 的 STBY 和额外四个 IN2 GPIO 不再占用。",
        "缺少该小板背面电路图与实测尺寸，图只作原理图/封装草案，不是可直接投板的 PCB。",
    ]
    for i, text in enumerate(notes):
        v.text(80, y + 83 + 37*i, "• " + text, 17, base.MUTED)


def main():
    rows = assignment()
    write_module_pinout()
    cn8, cn10 = v4.connector_rows(rows)
    base.SVG_PATH = OUT / (NAME + ".svg")
    base.PNG_PATH = OUT / (NAME + ".png")
    v = base.SVG(2600, 2200)
    v.text(55, 52, "DMG474 · 两块 AT8236 双路模块 / 两根牛角座排线", 31, base.INK, bold=True)
    v.text(55, 88, "四台12V 370电机；主板只连CN8/CN10，模块双排7针按照片标注。", 17, base.MUTED)
    base.draw_connector(v, 55, 135, "J1  ·  接主板 CN8 / PM1", cn8,
                        "CN8-23 5V留空；原TB6612增加的5脚未使用；IN1/IN2二线控制")
    base.draw_connector(v, 1355, 135, "J2  ·  接主板 CN10 / PM2", cn10,
                        "CN10-23的5V只送本地稳压；IPS走SPI3；RR_IN2在PC11")
    v.rect(1137, 875, 326, 86, "#f6fbf7", "#8fb9a0", 10)
    v.text(1300, 906, "本地3.3V / 共地", 17, base.GREEN, "middle", True)
    v.text(1300, 934, "5V: CN10-23 · GND: 7/16脚", 14, base.GROUND, "middle")
    draw_module(v, 55, 1095, "AT_FRONT", ("FL", "FR"))
    draw_module(v, 1355, 1095, "AT_REAR", ("RL", "RR"))
    draw_notes(v)
    v.save()


if __name__ == "__main__":
    main()
