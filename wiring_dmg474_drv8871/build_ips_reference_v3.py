"""Extend the verified motor connector reference with the IPS200 on CN3."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import build_connector_reference as base

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"


def write_v3_csv() -> None:
    source = OUT / "pin_assignment_v2.csv"
    with source.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        rows = list(reader)
    assert fields is not None
    extra = [
        ("IPS电源", "IPS_3V3", "无", "板载3.3V", "CN3", 2, "J_IPS.2", "需核对模块供电"),
        ("IPS地", "IPS_GND", "无", "GND", "CN3", 1, "J_IPS.1", "已核对"),
        ("IPS时钟", "IPS_SCK", "PB3", "SPI1_SCK", "CN3", 5, "J_IPS.3", "与板载SPI Flash共享时钟"),
        ("IPS数据", "IPS_MOSI", "PB5", "SPI1_MOSI", "CN3", 6, "J_IPS.4", "与板载SPI Flash共享数据"),
        ("IPS片选", "IPS_CS", "PD10", "GPIO", "CN3", 4, "J_IPS.5", "低有效"),
        ("IPS命令数据", "IPS_DC", "PD8", "GPIO", "CN3", 3, "J_IPS.6", "把原LCD_CE用作D/C"),
        ("IPS复位", "IPS_RST", "PD9", "GPIO", "CN3", 8, "J_IPS.7", "低有效"),
        ("IPS背光", "IPS_BL", "无", "经可选焊位接3.3V", "转接板", 0, "J_IPS.8", "实物核对限流与极性后装配"),
    ]
    for purpose, net, gpio, peripheral, connector, pin, target, status in extra:
        rows.append(dict(zip(fields, (purpose, net, "IPS200", gpio, peripheral,
                                      connector, str(pin), target, status))))
    active = [r for r in rows if r["MCU引脚"] != "无"]
    assert len(active) == len({r["MCU引脚"] for r in active}) == 23
    verified = json.loads((OUT / "connector_pinout_verified.json").read_text(encoding="utf-8"))["pins"]
    for r in active:
        if r["主板接口"] in verified:
            assert verified[r["主板接口"]][r["接口针号"]] == r["MCU引脚"], r
    with (OUT / "pin_assignment_v3_ips.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def draw_ips(v: base.SVG) -> None:
    y = 1545
    v.text(55, y - 20, "新增 IPS200 接口：由主板 CN3 单独引出，J1/J2 不占新脚", 24, base.INK, bold=True)
    v.rect(55, y, 2490, 235, "#f2fafc", "#7daeb9", 15)
    v.text(80, y + 39, "CN3  ·  主板 2×4 屏幕接口", 20, base.TEAL, bold=True)
    left = [("1  GND", "2  3V3"), ("3  PD8 → DC", "4  PD10 → CS"),
            ("5  PB3 → SCK", "6  PB5 → MOSI"),
            ("7  PB4 → NC", "8  PD9 → RST")]
    for i, (odd, even) in enumerate(left):
        v.text(80, y + 72 + 36 * i, odd, 18, base.GROUND if i == 0 else base.BLUE)
        v.text(310, y + 72 + 36 * i, even, 18, base.GREEN if i == 0 else base.BLUE)
    v.line(605, y + 115, 705, y + 115, base.BLUE, 3)
    v.text(750, y + 39, "J_IPS  ·  转接板自定义 8 针", 20, base.TEAL, bold=True)
    labels = ["1 GND", "2 3V3", "3 SCK", "4 MOSI", "5 CS", "6 DC", "7 RST", "8 BLK* "]
    for i, label in enumerate(labels):
        x = 750 + (i % 4) * 160
        yy = y + 79 + (i // 4) * 55
        v.rect(x, yy - 29, 135, 42, "#ffffff", "#94bdc4", 7)
        v.text(x + 11, yy, label, 18, base.TEAL)
    v.text(750, y + 200, "*BLK 由模块规格决定；预留可选供电/限流焊位", 17, base.RED)
    v.text(1520, y + 48, "PB3/PB5 与板载 SPI Flash 共用 SPI1", 19, base.INK, bold=True)
    v.text(1520, y + 85, "Flash 片选 PB8，屏幕片选 PD10；分时访问", 18, base.MUTED)
    v.text(1520, y + 122, "屏幕不接 MISO，CN3-7 保持空脚", 18, base.MUTED)
    v.text(1520, y + 159, "VCC、BLK 电压和模块插头顺序先核对实物", 18, base.RED)
    v.text(1520, y + 196, "原 E 工程 SPI2 引脚不可照搬到 DMG474", 18, base.RED)


def main() -> None:
    write_v3_csv()
    base.SVG_PATH = OUT / "05_双排座加IPS关系图_V3.svg"
    base.PNG_PATH = OUT / "05_双排座加IPS关系图_V3.png"
    v = base.SVG(2600, 2210)
    v.text(55, 52, "DMG474 · 四驱接口与 IPS200 原理图参考", 31, base.INK, bold=True)
    v.text(55, 88, "J1/J2 保持四驱分配；新增 CN3 → J_IPS。按网络标签画原理图，插头物理线序以实物为准。", 17, base.MUTED)
    base.draw_connector(v, 55, 135, "J1  ·  接主板 CN8 / PM1", base.CN8)
    base.draw_connector(v, 1355, 135, "J2  ·  接主板 CN10 / PM2", base.CN10)
    v.rect(1137, 875, 326, 86, "#f6fbf7", "#8fb9a0", 10)
    v.text(1300, 906, "公共参考", 17, base.GREEN, "middle", True)
    v.text(1300, 934, "GND: 7/16脚 · 3V3: CN2-1", 14, base.GROUND, "middle")
    base.draw_headers(v)
    draw_ips(v)
    base.draw_notes(v, y=1840)
    v.save()


if __name__ == "__main__":
    main()
