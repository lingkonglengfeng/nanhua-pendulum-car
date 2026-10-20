"""Adapter for selected complete wheelset; no guessed motor or axle height."""
from pathlib import Path
import json,math,zipfile
import cadquery as cq
import trimesh,ezdxf,resvg_py
R=Path(__file__).resolve().parent;O=R/'output';O.mkdir(parents=True,exist_ok=True)
def box(x,y,z,cx=0,cy=0,z0=0):return cq.Solid.makeBox(x,y,z,cq.Vector(cx-x/2,cy-y/2,z0))
def cyl(d,h,x,y,z=-1):return cq.Solid.makeCylinder(d/2,h,cq.Vector(x,y,z))
s=cq.Workplane(obj=box(72,60,3)).edges('|Z').fillet(3).val()
for x in [-28,28]:
    for y in [-18,18]:s=s.cut(cyl(4.5,5,x,y))
for x in [-8,8]:
    # Total slot 30x3.4, center Y=10. Allows a 14mm axial bolt pitch to slide.
    a=(30-3.4)/2
    cut=box(3.4,2*a,5,x,10,-1).fuse(cyl(3.4,5,x,10-a),cyl(3.4,5,x,10+a))
    s=s.cut(cut)
s=s.clean();assert s.isValid() and len(s.Solids())==1
for shift in [-6,0,6]:
    for x in [-8,8]:
        for y in [10+shift-7,10+shift+7]:assert s.intersect(cyl(3,5,x,y)).Volume()<1e-6
cq.exporters.export(s,str(O/'wheelset_adapter_14x16.step'))
cq.exporters.export(s.translate((36,30,0)),str(O/'wheelset_adapter_14x16.stl'),tolerance=.04,angularTolerance=.1)
m=trimesh.load_mesh(O/'wheelset_adapter_14x16.stl',process=True);assert m.is_watertight and m.is_volume
doc=ezdxf.new('R2010');doc.units=4;ms=doc.modelspace();q=math.tan(math.pi/8)
ms.add_lwpolyline([(-33,-30,0),(33,-30,q),(36,-27,0),(36,27,q),(33,30,0),(-33,30,q),(-36,27,0),(-36,-27,q)],format='xyb',close=True)
for x in [-28,28]:
    for y in [-18,18]:ms.add_circle((x,y),2.25)
for x in [-8,8]:
    a=(30-3.4)/2;r=1.7
    ms.add_lwpolyline([(x+r,10-a,0),(x+r,10+a,1),(x-r,10+a,0),(x-r,10-a,1)],format='xyb',close=True)
doc.saveas(O/'wheelset_adapter_14x16.dxf')
d=ezdxf.readfile(O/'wheelset_adapter_14x16.dxf');assert len(d.modelspace())==7 and not d.audit().has_errors
chassis=cq.importers.importStep(str(R.parent/'chassis_fr4_v1/output/step/01_chassis_FR4_320x220x4.step')).val()
for sx in [-1,1]:
    for sy in [-1,1]:
        for x in [-28,28]:
            for y in [-18,18]:assert chassis.intersect(cyl(4,7,sx*118+x,sy*80+y,-5)).Volume()<1e-6
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="880" viewBox="0 0 1200 880"><rect width="1200" height="880" fill="#f5f7fa"/><g font-family="Microsoft YaHei,sans-serif" fill="#173447">',
 '<text x="55" y="65" font-size="30" font-weight="600">新轮组专用转接片 · 14×16 mm</text>',
 '<text x="55" y="108" font-size="19">72×60×3 mm / 四轮需 4 片 / 接入现有主底板的 M4 可调接口</text>']
scale=7;cx=410;cy=420
svg.append(f'<rect x="{cx-36*scale}" y="{cy-30*scale}" width="{72*scale}" height="{60*scale}" rx="{3*scale}" fill="#e7eedc" stroke="#173447" stroke-width="2"/>')
for x in [-28,28]:
    for y in [-18,18]:svg.append(f'<circle cx="{cx+x*scale}" cy="{cy-y*scale}" r="{2.25*scale}" fill="white" stroke="#2469a0" stroke-width="2"/>')
for x in [-8,8]:
    svg.append(f'<rect x="{cx+(x-1.7)*scale}" y="{cy-25*scale}" width="{3.4*scale}" height="{30*scale}" rx="{1.7*scale}" fill="white" stroke="#b87a35" stroke-width="2"/>')
    for y in [3,17]:svg.append(f'<circle cx="{cx+x*scale}" cy="{cy-y*scale}" r="{1.5*scale}" fill="none" stroke="#c44444" stroke-width="2"/>')
for i,t in enumerate(['蓝色：4×Ø4.5，孔距 56×36','橙色：2×30×3.4 长槽','红圈：轮组四个 M3 螺丝位置','两槽横向中心距 16','同槽前后螺丝间距 14','沿电机轴向可移动约 ±6 mm']):svg.append(f'<text x="730" y="{235+i*52}" font-size="19">{t}</text>')
svg.extend(['<text x="55" y="725" font-size="20">适配已知孔距；轮轴高度、电机型号和电气参数仍待确认。</text>', '<text x="55" y="768" font-size="19">先调整轮胎与板边间隙再锁紧。推荐 FR4/铝板加工；STL 可用于试装。</text>', '<text x="55" y="815" font-size="18">加工用 DXF 的毫米尺寸。图中不把原 520 的 30×24 孔当作新轮组固定孔。</text>','</g></svg>'])
sv='\n'.join(svg);(O/'adapter_layout.svg').write_text(sv,encoding='utf-8')
(O/'adapter_layout.png').write_bytes(resvg_py.svg_to_bytes(svg_string=sv,font_files=['C:/Windows/Fonts/msyh.ttc']))
report={'part_size_mm':[72,60,3],'quantity':4,'wheelset_bolt_pattern_x_y':[16,14],'wheelset_hole_diameter_from_image':3.3,'chassis_interface_pattern':[56,36],'chassis_hole_match':True,'adjustment_centers_tested_mm':[-6,0,6],'valid_solid':True,'watertight_stl':True,'dxf_readback':True,'not_verified':['actual motor/encoder type and ratings','mounting-plane to axle height','wheel to bracket clearance','65 versus 66mm diameter labels','physical stiffness and grip']}
(O/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False))
