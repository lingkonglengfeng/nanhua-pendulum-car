"""SVG system wiring, deliberately no invented MCU pad/connector numbers."""
from pathlib import Path
from xml.sax.saxutils import escape
import json,csv,xml.etree.ElementTree as ET
import resvg_py

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output'
OUT.mkdir(exist_ok=True)
COL={'ink':'#213547','muted':'#5d7081','power':'#c33c37','low':'#298456','signal':'#2469a0','encoder':'#8552a1','ground':'#5e6872','pending':'#a46418'}
FONT='Microsoft YaHei'
class SVG:
    def __init__(self,w,h,title):
        self.w=w;self.h=h
        self.s=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',f'<title>{escape(title)}</title>',f'<rect width="{w}" height="{h}" fill="white"/>','<style>text{font-family:"Microsoft YaHei","Noto Sans CJK SC",sans-serif}path,line{stroke-linecap:round;stroke-linejoin:round}</style>']
    def text(self,x,y,t,size=21,color='ink',anchor='start',bold=False):
        self.s.append(f'<text x="{x}" y="{y}" font-size="{size}" fill="{COL.get(color,color)}" text-anchor="{anchor}" font-weight="{600 if bold else 400}">{escape(t)}</text>')
    def rect(self,x,y,w,h,fill='#fff',stroke='#bac8d3',dash=False,r=8):
        self.s.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="2"'+(' stroke-dasharray="8 6"' if dash else '')+'/>')
    def path(self,d,color='signal',width=3,dash=False):self.s.append(f'<path d="{d}" fill="none" stroke="{COL.get(color,color)}" stroke-width="{width}"'+(' stroke-dasharray="7 5"' if dash else '')+'/>')
    def line(self,x1,y1,x2,y2,color='signal',width=3,dash=False):self.path(f'M{x1},{y1} L{x2},{y2}',color,width,dash)
    def dot(self,x,y,color='signal',r=4):self.s.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{COL.get(color,color)}"/>')
    def port(self,x,y,color='signal'):
        self.s.append(f'<circle cx="{x}" cy="{y}" r="4" fill="white" stroke="{COL.get(color,color)}" stroke-width="2"/>')
    def net(self,x,y,label,color='signal',anchor='start',size=19):
        self.text(x,y,label,size,color,anchor,bold=True)
    def ground(self,x,y,label='GND'):
        self.line(x,y,x,y+12,'ground',2)
        for yy,ww in [(12,18),(18,12),(24,6)]:self.line(x-ww/2,y+yy,x+ww/2,y+yy,'ground',2)
        self.text(x+15,y+21,label,17,'ground')
    def bridge(self,x0,x1,y,crossings,color):
        d=f'M{x0},{y}'
        for x in sorted(crossings):
            if x0+8<x<x1-8:d+=f' H{x-7} Q{x},{y-13} {x+7},{y}'
        d+=f' H{x1}';self.path(d,color)
    def save(self,name):
        text='\n'.join(self.s+['</svg>']);p=OUT/(name+'.svg');p.write_text(text,encoding='utf-8');ET.parse(p)
        (OUT/(name+'.png')).write_bytes(resvg_py.svg_to_bytes(svg_path=str(p),font_family=FONT,font_files=['C:/Windows/Fonts/msyh.ttc']))
        return p

v=SVG(2000,1640,'倒立摆整车接线总图 ATK-DMG474 + 四路DRV8871')
v.text(65,60,'倒立摆整车接线总图',34,bold=True)
v.text(65,99,'按照片采用 ATK-DMG474；网络连接方案 V1 · 主控物理脚号尚未确认',22,'pending')
v.text(1930,65,'01 / 02',20,'muted','end')
# Battery, fuse and switches. S1 only removes motor supply.
v.rect(65,140,265,125,'#fff5f3','#dca49f');v.text(85,174,'B1  3S 电池',24,bold=True)
v.text(85,208,'11.1 V 标称 / 12.6 V 满电',18,'muted');v.text(85,239,'使用电池放电端',18,'muted')
v.port(330,170,'power');v.text(307,163,'+',22,'power');v.port(330,225,'ground');v.text(307,219,'−',22,'ground')
v.line(330,170,395,170,'power',4);v.rect(395,158,60,24,stroke=COL['power'],r=1);v.text(425,145,'F1',19,'power','middle')
v.line(455,170,515,170,'power',4);v.dot(515,170,'power');v.line(515,170,548,149,'power',3);v.dot(560,170,'power');v.line(560,170,700,170,'power',4)
v.text(515,211,'S0 总电源',18,'muted');v.dot(645,170,'power');v.line(645,170,645,280,'power',3);v.net(645,305,'VBAT_SW','power','middle')
v.text(365,248,'F1 按线材、峰值电流和时间特性选型',17,'muted')
v.dot(700,170,'power');v.line(700,170,742,146,'power');v.dot(755,170,'power');v.line(755,170,850,170,'power',4)
v.text(701,211,'S1 电机电源',18,'muted');v.net(850,146,'VM_MOTOR','power','middle')
v.line(850,170,850,1285,'power',4)
v.line(330,225,780,225,'ground',4);v.line(780,225,780,1285,'ground',4);v.dot(780,225,'ground');v.text(790,250,'GND_STAR',18,'ground')
# Explicit independent logic-peripheral regulator, not paralleled with MCU VDD.
v.rect(1010,140,350,170,'#f1f8f3','#9ec5ae')
v.text(1032,178,'U6 辅助 3.3 V 稳压模块',23,bold=True)
v.text(1032,212,'输入支持 3S；输出建议 ≥0.5 A',18,'muted')
v.text(1032,244,'供编码器、AS5600 和可选 IMU',18,'muted')
v.text(1032,278,'不与主板 3.3V 输出并联',18,'pending')
v.line(920,190,1010,190,'power');v.net(1000,132,'VBAT_SW','power','end',17)
v.line(1360,190,1450,190,'low');v.net(1465,197,'3V3_SENS','low')
v.ground(960,262);v.line(960,262,1010,262,'ground',2)
v.rect(1480,230,445,95,'#fff','#c4ced5');v.text(1500,264,'C1：电机母线储能电容',20,bold=True)
v.text(1500,295,'示例 470µF / 35V，跨接 VM_MOTOR 与 GND',17,'muted')
v.text(65,330,'实线连接或同名网络相连；跨线拱桥表示不连接。线色表示功能，不代表实物线色。',19,'muted')
# MCU role assignments, not invented GPIO pad numbers.
v.rect(65,390,595,900,'#f5f8fc','#6f92af')
v.text(92,431,'U1  ATK-DMG474',27,bold=True)
v.text(92,466,'需核对：原始 3.3V GPIO / 定时器 / I²C',20,'pending')
v.port(250,390,'power');v.line(250,370,250,390,'power');v.net(250,365,'DC输入正端 ← VBAT_SW','power','middle',18)
v.port(520,390,'ground');v.line(520,370,520,390,'ground');v.net(520,365,'DC输入负端 → GND','ground','middle',18)
CHANNELS=[('FL','左前'),('FR','右前'),('RL','左后'),('RR','右后')]
for i,(tag,zh) in enumerate(CHANNELS):
    y=400+i*220
    if i:v.line(85,y+75,640,y+75,'#d7e1e8',1)
    v.text(95,y+111,f'{i+1}  {zh} {tag}',22,bold=True)
    v.net(95,y+151,f'ENC_{tag}_A','encoder',size=19);v.text(250,y+151,'→ 计数输入 A',17,'muted')
    v.net(95,y+181,f'ENC_{tag}_B','encoder',size=19);v.text(250,y+181,'→ 计数输入 B',17,'muted')
    v.text(647,y+109,f'{tag}_PWM（待映射）',19,'signal','end')
    v.text(647,y+139,f'{tag}_DIR（待映射）',19,'signal','end')
    v.port(660,y+115);v.port(660,y+145)
    v.bridge(660,1010,y+115,[780,850],'signal');v.bridge(660,1010,y+145,[780,850],'signal')
    v.rect(1010,y,340,190,'#fff','#879baa')
    v.text(1030,y+28,f'U{i+2}  DRV8871 · {zh}',23,bold=True)
    v.dot(850,y+48,'power');v.line(850,y+48,1010,y+48,'power',3);v.port(1010,y+48,'power');v.text(1030,y+55,'POWER +',19,'power')
    v.dot(780,y+78,'ground');v.bridge(780,1010,y+78,[850],'ground');v.port(1010,y+78,'ground');v.text(1030,y+85,'POWER − / GND',19,'ground')
    v.port(1010,y+115);v.text(1030,y+122,'IN1',20,'signal');v.port(1010,y+145);v.text(1030,y+152,'IN2',20,'signal')
    for dy,num in [(115,1),(145,2)]:
        v.text(1330,y+dy+7,f'MOTOR {num}',18,'ink','end');v.port(1350,y+dy,'ink');v.line(1350,y+dy,1520,y+dy,'ink',3)
    v.text(1030,y+177,'排针 VCC 留空；不要接 3.3V',17,'pending')
    v.rect(1520,y,410,190,'#fbfcfd','#9eacb8')
    v.text(1540,y+29,f'M{i+1}  {zh}电机 + AB 编码器',23,bold=True)
    v.text(1540,y+60,'E+ → 3V3_SENS；E− → GND',18,'low')
    v.text(1540,y+88,f'A → ENC_{tag}_A；B → ENC_{tag}_B',18,'encoder')
    v.port(1520,y+115,'ink');v.text(1540,y+122,'电机线 A（按实物标识）',19)
    v.port(1520,y+145,'ink');v.text(1540,y+152,'电机线 B（按实物标识）',19)
    v.text(1540,y+177,'六根线：2 根电机线 + 4 根编码器线',16,'muted')
v.ground(780,1285,'公共地，电机负线独立回到分电点')
v.text(91,1270,'专用编码器口只有两组；另外两组需核对可用资源。',18,'pending')
# I2C role labels; exact pull-up and pin notes on sheet 2.
v.rect(65,1340,595,205,'#fffaf1','#cfad79');v.text(90,1379,'主控端尚待确认的内容',23,bold=True)
for j,t in enumerate(['4 路 PWM + 4 路方向 GPIO 的实际脚号','4 组 AB 输入的定时器/中断引脚与接口电平','I²C_SDA、I²C_SCL 的实际排针位置','不能把隔离/开漏端子直接等同于 3.3V GPIO']):v.text(90,1417+j*33,t,19,'pending')
v.rect(1010,1340,445,205,'#f4f8fc','#97b5cc');v.text(1035,1379,'AS5600 摆角模块',24,bold=True)
for j,(name,net,col) in enumerate([('VCC','3V3_SENS *','low'),('GND','GND','ground'),('SDA','I2C_SDA','signal'),('SCL','I2C_SCL','signal')]):v.text(1035,1415+j*29,f'{name} → {net}',20,col)
v.rect(1520,1340,410,205,'#fff','#97b5cc',True);v.text(1540,1379,'可选 IMU（预留）',24,bold=True)
v.text(1540,1420,'选择 3.3V I²C 模块',20);v.text(1540,1454,'并接 I2C_SDA / I2C_SCL',20,'signal');v.text(1540,1488,'电源接 3V3_SENS / GND',20,'low')
v.text(1010,1584,'* AS5600 模块须确认 3.3V 接法；GH1.25 插座针序不能按这张图的位置猜。',19,'pending')
v.text(65,1610,'本图确认模块之间的网络关系；主板原理图和模块针序核对前，不按照片端子位置直接接线。',20,'pending')
v.save('01_整车接线总图')

s=SVG(1900,1480,'DRV8871单路接线与AS5600细节')
s.text(60,60,'单路驱动与传感器接线细节',34,bold=True);s.text(60,99,'四路电机复制同一接法；以下引脚名称是功能名，不是排针位置顺序。',21,'muted');s.text(1840,65,'02 / 02',20,'muted','end')
s.rect(60,150,450,450,'#f5f8fc','#7f9db5');s.text(85,193,'主控原始 3.3V 信号',25,bold=True)
s.text(85,236,'Px 引脚 / 端子号：待官方原理图确认',20,'pending')
s.text(485,317,'PWM 输出',22,'signal','end');s.port(510,310);s.line(510,310,780,310)
s.text(485,397,'DIR 普通输出',22,'signal','end');s.port(510,390);s.line(510,390,780,390)
s.text(85,465,'主控与驱动共地',22);s.text(85,506,'电机大电流不经过主控排针',20,'pending');s.text(85,547,'转接 PCB 可选；端子排也能完成分配',18,'muted')
s.rect(780,150,430,450,'#fff','#7f9db5');s.text(805,193,'DRV8871 成品模块',25,bold=True)
s.port(780,240,'power');s.line(650,240,780,240,'power',4);s.net(645,232,'VM_MOTOR','power','end');s.text(805,248,'POWER +  → 电机电源',22,'power')
s.port(780,310);s.text(805,317,'IN1',24,'signal');s.port(780,390);s.text(805,397,'IN2',24,'signal')
s.port(780,465,'ground');s.line(690,465,780,465,'ground');s.ground(690,465);s.text(805,472,'POWER − / GND',22,'ground')
s.text(805,535,'排针 VCC：本方案不使用',21,'pending');s.text(805,570,'疑似与 POWER+ 同网，断电测通断确认',18,'muted')
for y,label in [(310,'MOTOR 1'),(390,'MOTOR 2')]:s.port(1210,y,'ink');s.line(1210,y,1430,y,'ink',4);s.text(1185,y-12,label,21,'ink','end')
s.rect(1430,150,410,450,'#fbfcfd','#9eacb8');s.text(1455,193,'370 编码电机',25,bold=True)
s.text(1455,251,'电机绕组',23,bold=True);s.port(1430,310,'ink');s.port(1430,390,'ink');s.text(1455,317,'电机线 A',22);s.text(1455,397,'电机线 B',22)
s.text(1455,466,'编码器供电与反馈：',21,bold=True);s.text(1455,505,'E+ = 3.3V；E− = GND',20,'low');s.text(1455,543,'A / B → 对应轮子的 ENC_A / ENC_B',18,'encoder');s.text(1455,579,'不要把电机电源接到编码器电源脚',18,'pending')
# Basic pin state table, and selected PWM+GPIO interpretation.
s.rect(60,640,800,320,'#fff','#b9c7d0');s.text(85,680,'输入逻辑（TI 数据手册）',24,bold=True)
rows=[('IN1','IN2','桥输出状态'),('0','0','高阻滑行；持续约 1 ms 后休眠'),('1','0','一个方向驱动'),('0','1','相反方向驱动'),('1','1','两输出拉低，制动')]
for i,row in enumerate(rows):
    yy=724+i*43
    for x,t in zip([100,200,305],row):s.text(x,yy,t,20,bold=i==0)
s.rect(900,640,940,320,'#fffaf1','#cfad79');s.text(925,680,'本图用 1 路 PWM + 1 路 GPIO 控制每个电机',24,bold=True)
for i,t in enumerate(['正向：IN1 = PWM(d)，IN2 = 0。','反向：IN1 = PWM(1−d)，IN2 = 1。','停止：统一令 IN1 = 0、IN2 = 0；需要制动时另行明确选择。','d 是驱动占空比，不等于实际转速百分比。','此方式正反衰减模式不同；换向需限斜率，限流按模块实装阻值核对。']):s.text(925,725+i*43,t,20,'ink' if i<4 else 'pending')
# I2C schematic with explicit pull-ups and rails.
s.text(60,1020,'AS5600 / 可选 IMU：I²C 接线',26,bold=True)
s.rect(60,1080,390,230,'#f5f8fc','#7f9db5');s.text(85,1118,'主控 I²C（脚号待定）',23,bold=True)
s.port(450,1170);s.port(450,1230);s.text(425,1177,'SDA',22,'signal','end');s.text(425,1237,'SCL',22,'signal','end')
s.line(450,1170,1280,1170,'signal',3);s.line(450,1230,1280,1230,'signal',3)
s.line(690,1080,920,1080,'low',3);s.net(805,1063,'3V3_SENS','low','middle')
for x,yy in [(720,1170),(890,1230)]:
    s.line(x,1080,x,1097,'low');s.rect(x-10,1097,20,42,stroke=COL['low'],r=0)
    if yy==1230:s.path(f'M{x},1139 V1163 Q{x+13},1170 {x},1177 V{yy}','low')
    else:s.line(x,1139,x,yy,'low')
    s.dot(x,yy,'signal');s.text(x+22,1120,'4.7kΩ *',18,'low')
s.rect(1280,1060,560,300,'#f4f8fc','#97b5cc');s.text(1305,1102,'AS5600 模块 / 逻辑功能端',24,bold=True)
s.port(1280,1170);s.text(1305,1177,'SDA',23,'signal');s.port(1280,1230);s.text(1305,1237,'SCL',23,'signal')
s.text(1510,1153,'VCC → 3V3_SENS **',20,'low');s.text(1510,1190,'GND → GND',20,'ground');s.text(1305,1288,'I²C 地址 0x36；与可选 IMU 同总线时核对地址冲突。',18,'muted');s.text(1305,1325,'GH 插头的实际四针顺序由卖家接线图确认。',19,'pending')
s.text(60,1360,'* 已有上拉时先核对阻值及上拉电压，避免重复添加；总线上拉到 3.3V。',19,'muted')
s.text(60,1400,'** 模块电路须允许 3.3V 供电；若只能 5V，需另设计供电/电平处理，不能照本图直接接。',19,'pending')
s.text(60,1440,'模块内的限流、电容及针序因厂家而异；芯片功能已查证，具体小板布线仍以实物/原理图为准。',19,'muted')
s.save('02_单路驱动与传感器细节')

# Machine-readable signal list: physical MCU pads deliberately remain TBD.
nets={}
def net(name,*pins):nets.setdefault(name,[]).extend(pins)
net('BAT_POS','B1.+','F1.1');net('FUSED_POS','F1.2','S0.1')
net('VBAT_SW','S0.2','S1.1','U1.DC_IN_POS','U6.VIN_POS')
net('VM_MOTOR','S1.2','C1.POS')
net('GND','B1.-','U1.DC_IN_NEG','U6.VIN_NEG','U6.VOUT_NEG','C1.NEG','AS5600.GND','IMU_OPTIONAL.GND')
net('3V3_SENS','U6.VOUT_3V3','AS5600.VCC_CONDITIONAL','IMU_OPTIONAL.VCC_3V3','RP_SDA.1','RP_SCL.1')
net('I2C_SDA','U1.TBD_SDA','AS5600.SDA','IMU_OPTIONAL.SDA','RP_SDA.2')
net('I2C_SCL','U1.TBD_SCL','AS5600.SCL','IMU_OPTIONAL.SCL','RP_SCL.2')
rows=[]
for i,(tag,zh) in enumerate(CHANNELS):
    d=f'DRV{i+1}';m=f'M{i+1}'
    net('VM_MOTOR',d+'.POWER_POS');net('GND',d+'.POWER_NEG',d+'.GND_HEADER',m+'.ENC_GND')
    net('3V3_SENS',m+'.ENC_VCC')
    net(tag+'_PWM',f'U1.TBD_PWM_{tag}',d+'.IN1');net(tag+'_DIR',f'U1.TBD_DIR_{tag}',d+'.IN2')
    net('ENC_'+tag+'_A',m+'.ENC_A',f'U1.TBD_ENC_{tag}_A');net('ENC_'+tag+'_B',m+'.ENC_B',f'U1.TBD_ENC_{tag}_B')
    net(tag+'_MOTOR_1',d+'.MOTOR1',m+'.MOTOR_A');net(tag+'_MOTOR_2',d+'.MOTOR2',m+'.MOTOR_B')
    rows.append({'wheel':zh,'tag':tag,'schematic_ref':f'U{i+2}','driver':d,'IN1':tag+'_PWM','IN2':tag+'_DIR','encoder_A':'ENC_'+tag+'_A','encoder_B':'ENC_'+tag+'_B','physical_MCU_pins':'TBD — 需 ATK-DMG474 主板原理图'})
seen={}
for n,pins in nets.items():
    for p in pins:
        assert p not in seen,(p,n,seen.get(p));seen[p]=n
for i in range(1,5):
    assert seen[f'DRV{i}.POWER_POS']=='VM_MOTOR'
    assert seen[f'M{i}.ENC_VCC']=='3V3_SENS'
    assert f'DRV{i}.VCC_HEADER' not in seen
assert 'U1.3V3_OUT' not in seen
data={'status':'functional_wiring_only_MCU_pins_unassigned','board_from_photo':'ATK-DMG474','nets':nets,'no_connect':[f'DRV{i}.VCC_HEADER' for i in range(1,5)],'conditions':['U1 raw 3.3V GPIO and encoder input availability must be verified','AS5600 module must support 3.3V configuration','Encoder A/B electrical levels and pull-ups need module check','C1 value and fusing illustrative, not validated power-transient design']}
(OUT/'netlist.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
with (OUT/'channel_assignment.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
(OUT/'verification.json').write_text(json.dumps({'svg_xml_parse':True,'svg_png_render':True,'four_independent_motor_channels':True,'driver_VCC_headers_not_connected':True,'sensor_supply_not_motor_rail':True,'external_3V3_not_paralleled_with_main_3V3':True,'logical_net_duplicates':False,'physical_pin_map_validated':False},indent=2),encoding='utf-8')
print('Wrote 2 SVGs, 2 PNGs, logical netlist and channel table.')
