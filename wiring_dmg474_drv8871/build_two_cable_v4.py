"""Two-ribbon DMG474 connector reference: four drives, sensors, and IPS200."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import build_connector_reference as base

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"
NAME = "06_双牛角座全接线_V4"


def assignment() -> list[dict[str, str]]:
    with (OUT / "pin_assignment_v2.csv").open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        rows = list(reader)
    assert fields is not None
    for r in rows:
        if r["网络"] == "RR_DIR":
            r.update({"MCU引脚": "PC11", "主板接口": "CN10", "接口针号": "20",
                      "状态": "为SPI3_SCK释放PC10"})
        elif r["网络"] == "3V3_SENS":
            r.update({"外设": "本地5V转3.3V稳压输出", "主板接口": "转接板", "接口针号": "0",
                      "状态": "与主板3.3V不并联；按负载选稳压器"})
        elif r["网络"] == "GND":
            r.update({"主板接口": "CN10", "接口针号": "7",
                      "状态": "CN8/CN10的7/16脚同为信号地；不承载电机回流"})

    extra = [
        ("5V输入", "VCC5_TO_REG", "无", "主板VCC5", "CN10", 23, "U_REG.VIN", "按总负载预算核算；CN8-23留空"),
        ("IPS电源", "IPS_3V3", "无", "本地3.3V输出", "转接板", 0, "J_IPS.VCC", "确认屏幕3.3V兼容"),
        ("IPS时钟", "IPS_SCK", "PC10", "SPI3_SCK", "CN10", 18, "J_IPS.SCK", "已核对芯片复用和主板原理图"),
        ("IPS数据", "IPS_MOSI", "PC12", "SPI3_MOSI", "CN10", 22, "J_IPS.MOSI", "屏幕单向发送不用MISO"),
        ("IPS片选", "IPS_CS", "PD11", "GPIO", "CN10", 24, "J_IPS.CS", "低有效"),
        ("IPS命令数据", "IPS_DC", "PD14", "GPIO", "CN10", 5, "J_IPS.DC", "普通输出"),
        ("IPS复位", "IPS_RST", "PD7", "GPIO", "CN10", 6, "J_IPS.RST", "低有效"),
        ("IPS背光", "IPS_BL", "无", "按模块规格可选本地供电", "转接板", 0, "J_IPS.BLK", "先核对背光限流和极性"),
    ]
    for purpose, net, gpio, peripheral, connector, pin, target, status in extra:
        rows.append(dict(zip(fields, (purpose, net, "IPS200/转接板", gpio, peripheral,
                                      connector, str(pin), target, status))))
    signals = [r for r in rows if r["MCU引脚"] != "无"]
    assert len(signals) == len({r["MCU引脚"] for r in signals}) == 23
    truth = json.loads((OUT / "connector_pinout_verified.json").read_text(encoding="utf-8"))["pins"]
    for r in signals:
        if r["主板接口"] in truth:
            assert truth[r["主板接口"]][r["接口针号"]] == r["MCU引脚"], r
    with (OUT / "pin_assignment_v4_two_cables.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def connector_rows(rows: list[dict[str, str]]) -> tuple[list[tuple[int, str, str]], list[tuple[int, str, str]]]:
    nets = {(r["主板接口"], int(r["接口针号"])): r["网络"]
            for r in rows if r["主板接口"] in ("CN8", "CN10")}
    # Drive the drawing from the validated CSV rather than a second pin list.
    truth = json.loads((OUT / "connector_pinout_verified.json").read_text(encoding="utf-8"))["pins"]
    result = []
    for connector in ("CN8", "CN10"):
        result.append([(pin, truth[connector][str(pin)],
                        "GND" if truth[connector][str(pin)] == "GND" else
                        nets.get((connector, pin), "NC")) for pin in range(1, 25)])
    return result[0], result[1]


def draw_screen_and_power(v: base.SVG) -> None:
    y = 1545
    v.text(55, y - 20, "屏幕占用 CN10 空脚；右后轮 DIR 改到 PC11 / CN10-20", 24, base.INK, bold=True)
    v.rect(55, y, 2490, 225, "#f2fafc", "#7daeb9", 15)
    v.text(80, y + 40, "IPS200 · SPI3 单向发送", 21, base.TEAL, bold=True)
    screen = [("SCK", "PC10 / 18"), ("MOSI", "PC12 / 22"),
              ("CS", "PD11 / 24"), ("DC", "PD14 / 5"), ("RST", "PD7 / 6")]
    for i, (signal, pin) in enumerate(screen):
        row = i % 3
        col = i // 3
        v.text(80 + col * 300, y + 78 + row * 39,
               f"{signal}  ← CN10-{pin.split(' / ')[1]}  {pin.split(' / ')[0]}", 18, base.BLUE)
    v.text(720, y + 42, "J_IPS · 本板自定义 8 针", 20, base.TEAL, bold=True)
    v.text(720, y + 80, "1 GND    2 3V3    3 SCK    4 MOSI", 18, base.INK)
    v.text(720, y + 120, "5 CS       6 DC       7 RST      8 BLK*", 18, base.INK)
    v.text(720, y + 164, "*背光接法按模块实物；不直接假设限流", 17, base.RED)
    v.text(1440, y + 42, "CN10-23  5V → U_REG → 3V3_LOCAL", 20, base.GREEN, bold=True)
    v.text(1440, y + 81, "本地 3.3V → 编码器 / AS5600 / IPS", 18, base.GREEN)
    v.text(1440, y + 120, "U_REG 输入/输出电容按选定芯片数据手册", 17, base.MUTED)
    v.text(1440, y + 159, "仅信号与传感器电流；电机大电流单独分电", 17, base.RED)
    v.text(1440, y + 197, "主板 CN2 / CN3 均无需排线", 18, base.TEAL, bold=True)


def draw_v4_notes(v: base.SVG) -> None:
    y = 1835
    v.rect(55, y, 2490, 325, "#fffaf4", "#d3ae78", 14)
    v.text(78, y + 36, "上电前与原理图绘制注意", 22, base.RED, bold=True)
    notes = [
        "主板仅连 CN8→J1、CN10→J2 两根 24 芯 1:1 排线；两个牛角座的 1 脚、防呆口和排线方向实物核对。",
        "CN10-23 是主板 5V，单点喂 U_REG；CN8-23 留空。U_REG 的 3.3V 不并接主板 CN2-1。",
        "CN8/CN10 的 7、16 脚均为 GND；逻辑/传感器共地，电机 POWER− 走独立粗线回分电点。",
        "I²C4 仍在 PC6/PC7；它们还接板载步进/舵机电路，断开 P7/CN14 步进供电并测波形。",
        "IPS 模块 VCC、BLK、插针顺序和总电流待实物核对；稳压器容量及散热按实际负载选择。",
        "DRV8871 的 IN1/IN2: 10正转、01反转、00滑行、11刹车；*_DIR是IN2控制网名，模块VM针非3.3V。",
    ]
    for i, item in enumerate(notes):
        v.text(78, y + 75 + i * 39, "• " + item, 17, base.MUTED)


def main() -> None:
    rows = assignment()
    cn8, cn10 = connector_rows(rows)
    base.SVG_PATH = OUT / (NAME + ".svg")
    base.PNG_PATH = OUT / (NAME + ".png")
    v = base.SVG(2600, 2200)
    v.text(55, 52, "DMG474 · 两根牛角座排线完成四驱 + IPS200", 31, base.INK, bold=True)
    v.text(55, 88, "无 CN2/CN3 排线；屏幕改由 CN10 的 SPI3 驱动，转接板从 5V 生成本地 3.3V。", 17, base.MUTED)
    base.draw_connector(v, 55, 135, "J1  ·  接主板 CN8 / PM1", cn8)
    base.draw_connector(v, 1355, 135, "J2  ·  接主板 CN10 / PM2", cn10,
                        "NC 画不连接叉号；CN10-23 的5V只送本地稳压器")
    v.rect(1137, 875, 326, 86, "#f6fbf7", "#8fb9a0", 10)
    v.text(1300, 906, "本地3.3V / 共地", 17, base.GREEN, "middle", True)
    v.text(1300, 934, "5V: CN10-23  ·  GND: 7/16脚", 14, base.GROUND, "middle")
    base.draw_headers(v)
    draw_screen_and_power(v)
    draw_v4_notes(v)
    v.save()


if __name__ == "__main__":
    main()
