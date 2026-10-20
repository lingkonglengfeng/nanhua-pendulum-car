from pathlib import Path
import ast,json,math,csv,zipfile
import cadquery as cq
import numpy as np
import trimesh,ezdxf,resvg_py
R=Path(__file__).resolve().parent;OUT=R/'output';V=cq.Vector
for f in ['step','dxf','stl_reference','preview']:(OUT/f).mkdir(parents=True,exist_ok=True)
# Reuse pure geometry/export functions only; do not execute or modify V1.
old=R.parent/'chassis_fr4_v1'
tree=ast.parse((old/'build_chassis.py').read_text(encoding='utf-8'))
names={'box','cylinder','slot_shape','rounded_plate','cut_for','rounded_outline','dxf_file'}
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[]),'geometry_helpers','exec'))
features=json.loads((old/'output/hole_features.json').read_text(encoding='utf-8'))
features=[h for h in features if h['group'] not in ['MOTOR_M4','MOTOR520_M3']]
for sx in [-1,1]:
    for sy in [-1,1]:
        for dx in [-8,8]:
            for dy in [-7,7]:features.append({'group':'WHEELSET_DIRECT_M3','kind':'circle','x':sx*118+dx,'y':sy*94+dy,'diameter':3.4})
for i,h in enumerate(features,1):h['id']=f'H{i:03d}'
assert len(features)==84
plate=rounded_plate(320,220,4,-4,10)
cuts=[cut_for(h) for h in features]
for c in cuts:plate=plate.cut(c)
plate=plate.clean();assert plate.isValid() and len(plate.Solids())==1
for i,a in enumerate(cuts):
    for b in cuts[i+1:]:assert a.intersect(b).Volume()<1e-6
name='chassis_V2_direct_14x16_320x220x4'
cq.exporters.export(plate,str(OUT/'step'/f'{name}.step'))
cq.exporters.export(plate.translate((160,110,4)),str(OUT/'stl_reference'/f'{name}.stl'),tolerance=.06,angularTolerance=.12)
m=trimesh.load_mesh(OUT/'stl_reference'/f'{name}.stl',process=True);assert m.is_watertight and m.is_volume
dxf_file(name,320,220,10,features)
assert cq.importers.importStep(str(OUT/'step'/f'{name}.step')).val().isValid()
base=cq.importers.importStep(str(R.parent/'pendulum_v3/output/step/01_base_shoe.step')).val()
for sy in [-1,1]:
    placed=base.translate((0,sy*65,0));assert placed.intersect(plate).Volume()<1e-6
    for x in [-55,55]:
        for yy in [-23,23]:assert placed.intersect(cylinder(5,12,x,sy*65+yy,-1)).Volume()<1e-6
for cx in [-118,118]:
    for cy in [-94,94]:
        for dx in [-8,8]:
            for dy in [-7,7]:assert plate.intersect(cylinder(3,6,cx+dx,cy+dy,-5)).Volume()<1e-6
with (OUT/'hole_coordinates_mm.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['编号','用途','形状','X','Y','直径','总长','槽宽','角度'])
    for h in features:w.writerow([h['id'],h['group'],h['kind'],h['x'],h['y'],h.get('diameter',''),h.get('length',''),h.get('width',''),h.get('angle','')])
(OUT/'hole_features.json').write_text(json.dumps(features,ensure_ascii=False,indent=2),encoding='utf-8')
# Actual-scale SVG drawing in mm, with dimension/purpose labels in the margins.
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="1060" viewBox="0 0 1400 1060"><rect width="1400" height="1060" fill="white"/><g font-family="Microsoft YaHei,sans-serif" fill="#213547">',
 '<text x="60" y="60" font-size="30" font-weight="600">底板 V2 · 新轮组直接安装</text>',
 '<text x="60" y="104" font-size="21">320×220×4 mm · 四组 16(X)×14(Y) 孔距 · 每组四个 Ø3.4 孔</text>']
sc=3.4;ox=680;oy=510
svg.append(f'<rect x="{ox-160*sc}" y="{oy-110*sc}" width="{320*sc}" height="{220*sc}" rx="{10*sc}" fill="#f4f6ef" stroke="#213547" stroke-width="2"/>')
for sy in [-1,1]:svg.append(f'<rect x="{ox-80*sc}" y="{oy-(sy*65+30)*sc}" width="{160*sc}" height="{60*sc}" fill="#f3d3ac" fill-opacity="0.25" stroke="#b07e46" stroke-dasharray="7 5"/>')
for h in features:
    x,y=ox+h['x']*sc,oy-h['y']*sc;c='#c14141' if h['group']=='WHEELSET_DIRECT_M3' else '#246e87'
    if h['kind']=='circle':svg.append(f'<circle cx="{x}" cy="{y}" r="{h["diameter"]/2*sc}" fill="white" stroke="{c}" stroke-width="2"/>')
    else:
        l,w=h['length']*sc,h['width']*sc
        svg.append(f'<rect x="{-l/2}" y="{-w/2}" width="{l}" height="{w}" rx="{w/2}" transform="translate({x},{y}) rotate({-h["angle"]})" fill="white" stroke="{c}" stroke-width="1.5"/>')
for x in [-118,118]:
    for y in [-94,94]:
        xx,yy=ox+x*sc,oy-y*sc
        svg.append(f'<rect x="{xx-14*sc}" y="{yy-13*sc}" width="{28*sc}" height="{26*sc}" fill="none" stroke="#c14141" stroke-width="1.5" stroke-dasharray="5 4"/>')
svg.extend(['<text x="60" y="925" font-size="22" fill="#c14141">红孔：整套新轮组直接固定；不需要独立小转接片。</text>',
 '<text x="60" y="969" font-size="20">保留原倒立摆、板卡、IMU、电池和传感器接口；原 520 / 转接片孔已移除。</text>',
 '<text x="60" y="1014" font-size="19">孔位已检查；轴心高度和实际轮胎间隙仍需实物确认。加工请使用 DXF 毫米尺寸。</text>','</g></svg>'])
s='\n'.join(svg);(OUT/'preview/chassis_V2_top.svg').write_text(s,encoding='utf-8')
(OUT/'preview/chassis_V2_top.png').write_bytes(resvg_py.svg_to_bytes(svg_string=s,font_files=['C:/Windows/Fonts/msyh.ttc']))
report={'direct_mount':True,'feature_count':84,'motor_holes':16,'motor_hole_pitch_x_y_mm':[16,14],'motor_mount_centers_xy_mm':[[x,y] for x in [-118,118] for y in [-94,94]],'motor_hole_diameter_mm':3.4,'solid_valid':True,'stl_closed':True,'dxf_readback':True,'old_pendulum_base_compatible':True,'hole_cutters_overlap':False,'axle_height_confirmed':False,'physical_fit_tested':False}
(OUT/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report))
