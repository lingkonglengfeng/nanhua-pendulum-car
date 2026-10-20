"""TB6612 two-module reference using DMG474 CN8/CN10 only."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import build_connector_reference as base
import build_two_cable_v4 as v4

OUT = Path(__file__).resolve().parent / "output"
NAME = "07_TB6612双牛角座接线_V5"
EXTRA = {
    "FL_IN2": ("左前", "PC13", "CN8", "24", "TB_FRONT.AIN2"),
    "FR_IN2": ("右前", "PE4", "CN8", "5", "TB_FRONT.BIN2"),
    "RL_IN2": ("左后", "PA2", "CN8", "6", "TB_REAR.AIN2"),
    "RR_IN2": ("右后", "PA3", "CN8", "8", "TB_REAR.BIN2"),
    "TB_STBY": ("两片TB6612", "PC0", "CN8", "15", "TB_FRONT.STBY+TB_REAR.STBY"),
}


def assignment():
    with (OUT / "pin_assignment_v4_two_cables.csv").open(encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        rows = list(reader)
    assert fields is not None
    pairs = {"FL": ("FRONT", "A"), "FR": ("FRONT", "B"),
             "RL": ("REAR", "A"), "RR": ("REAR", "B")}
    for r in rows:
        net = r["网络"]
        tag = net[:2]
        if tag not in pairs:
            continue
        chip, channel = pairs[tag]
        if net == f"{tag}_PWM":
            r.update({"目标端": f"TB_{chip}.PWM{channel}", "状态": "TB6612标准PWM脚"})
        elif net == f"{tag}_DIR":
            r.update({"网络": f"{tag}_IN1", "目标端": f"TB_{chip}.{channel}IN1",
                      "状态": "普通GPIO；与IN2构成方向"})
    for net, (wheel, gpio, connector, pin, target) in EXTRA.items():
        rows.append(dict(zip(fields, ("TB6612控制", net, wheel, gpio, "GPIO",
                                      connector, pin, target, "已核对主板座子"))))
    signals = [r for r in rows if r["MCU引脚"] != "无"]
    assert len(signals) == len({r["MCU引脚"] for r in signals}) == 28
    truth = json.loads((OUT / "connector_pinout_verified.json").read_text(encoding="utf-8"))["pins"]
    for r in signals:
        if r["主板接口"] in truth:
            assert truth[r["主板接口"]][r["接口针号"]] == r["MCU引脚"], r
    with (OUT / "pin_assignment_v5_tb6612.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def draw_tb(v):
    v.text(55, 1090, "两片双路 TB6612FNG：每台电机 PWM + IN1 + IN2；共用 STBY", 24, base.INK, bold=True)
    for i, (name, tags) in enumerate((("TB_FRONT", ("FL", "FR")),
                                       ("TB_REAR", ("RL", "RR")))):
        x, y = 55 + i * 1300, 1115
        v.rect(x, y, 1190, 350, "#f3f9fc", "#8caec8", 15)
        v.text(x + 25, y + 39, f"U{i+1} · {name}（两路有刷电机）", 23, base.BLUE, bold=True)
        v.text(x + 25, y + 76, "双排排针只按网络连接；实物排列、针距与封装待核对", 17, base.RED)
        v.text(x + 25, y + 115, "VM ← 3S电机母线    VCC ← 3V3_LOCAL    GND ← 公共地", 18, base.INK)
        v.text(x + 25, y + 147, "STBY ← PC0 / CN8-15（10k下拉，上电默认禁用）", 18, base.GREEN)
        for j, (channel, tag) in enumerate(zip(("A", "B"), tags)):
            yy = y + 189 + j * 65
            v.text(x + 25, yy, f"{channel}路 {tag}: PWM{channel}←{tag}_PWM   {channel}IN1←{tag}_IN1   {channel}IN2←{tag}_IN2", 18, base.BLUE)
            v.text(x + 25, yy + 29, f"{channel}O1/{channel}O2 → {tag} 电机绕组；编码器另接 J_ENC_{tag}", 16, base.MUTED)
        v.text(x + 25, y + 339, "故障：STBY=0；PWM高时，10/01=正反转，00=滑行，11=刹车", 16, base.RED)


def draw_sensors(v):
    y = 1505
    v.text(55, y - 17, "编码器与 AS5600 接口保持不变；电源来自本地 3.3V", 22, base.INK, bold=True)
    for i, tag in enumerate(("FL", "FR", "RL", "RR")):
        x = 55 + i * 480
        v.rect(x, y, 440, 140, "#fbf7ff", "#b195c2", 12)
        v.text(x + 16, y + 30, f"J_ENC_{tag}", 18, base.PURPLE, bold=True)
        v.text(x + 16, y + 67, "3V3_LOCAL / GND", 16, base.GREEN)
        v.text(x + 16, y + 103, f"ENC_{tag}_A / ENC_{tag}_B", 16, base.PURPLE)
    v.rect(1980, y, 565, 140, "#f4fbfb", "#83b7b5", 12)
    v.text(1998, y + 30, "J_AS5600 · I²C4", 18, base.TEAL, bold=True)
    v.text(1998, y + 67, "3V3_LOCAL / GND", 16, base.GREEN)
    v.text(1998, y + 103, "PC6 SCL / PC7 SDA", 16, base.TEAL)


def draw_notes(v):
    y = 1680
    v.rect(55, y, 2490, 180, "#f2fafc", "#7daeb9", 14)
    v.text(80, y + 40, "IPS200 · SPI3 仍走 CN10 空脚", 21, base.TEAL, bold=True)
    v.text(80, y + 79, "SCK PC10/18 · MOSI PC12/22 · CS PD11/24 · DC PD14/5 · RST PD7/6", 19, base.BLUE)
    v.text(80, y + 119, "CN10-23 5V → 本地稳压 3.3V → TB6612 VCC、编码器、AS5600、IPS", 18, base.GREEN)
    v.text(80, y + 153, "TB6612 的 VM 独走 3S 电机电源；两片共用 STBY，低电平禁用", 17, base.RED)
    y = 1890
    v.rect(55, y, 2490, 275, "#fffaf4", "#d3ae78", 14)
    v.text(80, y + 39, "投板前关键核对", 22, base.RED, bold=True)
    notes = [
        "12V/280rpm 电机：额定0.45A、堵转1.3A；卖家注明不可堵转或带负载瞬间反转。",
        "TB6612 每路平均1.2A、峰值3.2A/10ms；两路共用芯片散热，软件应限幅并避免持续堵转。",
        "3S满电12.6V距 TB6612 推荐 VM 上限13.5V 仅0.9V，实测制动/换向时的母线尖峰。",
        "两片模块双排针的针距、1脚、排列、VM/VCC/STBY/PWM/IN1/IN2 和电机输出须按实物核对。",
        "CN8/CN10 只作信号及本地低压取电；四台电机功率回路不走牛角座或转接板细铜线。",
    ]
    for i, item in enumerate(notes):
        v.text(80, y + 80 + i * 38, "• " + item, 17, base.MUTED)


def main():
    rows = assignment()
    cn8, cn10 = v4.connector_rows(rows)
    base.SVG_PATH = OUT / (NAME + ".svg")
    base.PNG_PATH = OUT / (NAME + ".png")
    v = base.SVG(2600, 2200)
    v.text(55, 52, "DMG474 · 双路 TB6612 两片 / 两根牛角座排线", 31, base.INK, bold=True)
    v.text(55, 88, "两片共驱四台 12V 370 电机；主板到转接板仍只连 CN8 和 CN10。", 17, base.MUTED)
    base.draw_connector(v, 55, 135, "J1  ·  接主板 CN8 / PM1", cn8,
                        "CN8-23 5V留空；PC0 控制 STBY；每电机 PWM/IN1/IN2")
    base.draw_connector(v, 1355, 135, "J2  ·  接主板 CN10 / PM2", cn10,
                        "CN10-23 的5V只送本地稳压；屏幕 SPI3；RR_IN1 在 PC11")
    v.rect(1137, 875, 326, 86, "#f6fbf7", "#8fb9a0", 10)
    v.text(1300, 906, "本地3.3V / 共地", 17, base.GREEN, "middle", True)
    v.text(1300, 934, "5V: CN10-23  ·  GND: 7/16脚", 14, base.GROUND, "middle")
    draw_tb(v)
    draw_sensors(v)
    draw_notes(v)
    v.save()


if __name__ == "__main__":
    main()
