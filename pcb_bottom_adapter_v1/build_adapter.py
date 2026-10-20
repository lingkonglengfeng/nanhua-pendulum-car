"""Parametric underside PCB adapter. Units: mm. No source models are modified."""
from pathlib import Path
import json, math, csv, sys, zipfile
import cadquery as cq
import numpy as np
import trimesh, ezdxf, resvg_py
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parent; BASE=ROOT.parent; OUT=ROOT/'output'
P=json.loads((ROOT/'parameters.json').read_text(encoding='utf-8'))
for d in ['step','stl','preview']: (OUT/d).mkdir(parents=True,exist_ok=True)
V=cq.Vector
def box(w,h,t,x=0,y=0,z=0): return cq.Solid.makeBox(w,h,t,V(x-w/2,y-h/2,z))
def cyl(d,h,x=0,y=0,z=0): return cq.Solid.makeCylinder(d/2,h,V(x,y,z))
def rounded(w,h,t,r,z=0): return cq.Workplane(obj=box(w,h,t,z=z)).edges('|Z').fillet(r).val()
def pattern(pitch): return [(x,y) for x in [-pitch[0]/2,pitch[0]/2] for y in [-pitch[1]/2,pitch[1]/2]]
pcbholes=pattern(P['pcb_hole_pitch_xy_mm']); mounts=pattern(P['chassis_hole_pitch_xy_mm'])
hole=P['clearance_hole_diameter_mm']; t=P['adapter_plate_thickness_mm']; bh=P['pcb_boss_height_mm']
w,h=P['adapter_outline_xy_mm']; wh=P['window_xy_mm']
plate=rounded(w,h,t,P['adapter_corner_radius_mm'])
window=rounded(*wh,t+2,P['window_corner_radius_mm'],z=-1)
plate=plate.cut(window)
for x,y in pcbholes:
    root=cq.Solid.makeCone(P['pcb_boss_root_diameter_mm']/2,P['pcb_boss_seat_diameter_mm']/2,2,V(x,y,t))
    plate=plate.fuse(root,cyl(P['pcb_boss_seat_diameter_mm'],bh-2,x,y,t+2))
for x,y in mounts+pcbholes: plate=plate.cut(cyl(hole,t+bh+2,x,y,-1))
plate=plate.clean()
spacer=cyl(P['chassis_spacer_diameter_mm'],P['chassis_spacer_height_mm']).cut(cyl(hole,12,z=-1)).clean()

def export_part(name,shape):
    assert shape.isValid() and len(shape.Solids())==1
    cq.exporters.export(shape,str(OUT/'step'/f'{name}.step'))
    cq.exporters.export(shape,str(OUT/'stl'/f'{name}.stl'),tolerance=.035,angularTolerance=.10)
    mesh=trimesh.load_mesh(OUT/'stl'/f'{name}.stl',process=True)
    assert mesh.is_watertight and mesh.is_volume
    reread=cq.importers.importStep(str(OUT/'step'/f'{name}.step')).val()
    assert reread.isValid() and len(reread.Solids())==1
    assert abs(reread.Volume()-shape.Volume())<.001
    return {'valid_solid':True,'watertight_stl':True,'step_readback':True,'volume_cm3':round(shape.Volume()/1000,3)}
checks={'adapter':export_part('01_adapter_84x64_M3',plate),'spacer':export_part('02_spacer_D8_H10_M3_qty4',spacer)}

# Assembly coordinates: deck upper surface Z=0, underside Z=-4.
cx,cy=P['installation_center_xy_mm']; upper=-4-P['chassis_spacer_height_mm']; adapter_offset=upper-t
def installed(s): return s.rotate((0,0,0),(1,0,0),180).translate((cx,cy,upper))
deck=cq.importers.importStep(str(BASE/P['chassis_step'])).val()
adapter=installed(plate)
pcbtop=adapter_offset-bh; pcbth=P['pcb_thickness_reference_mm']
pcb=box(*P['pcb_outline_xy_mm'],pcbth,cx,cy,pcbtop-pcbth)
for x,y in pcbholes: pcb=pcb.cut(cyl(hole,pcbth+2,cx+x,cy+y,pcbtop-pcbth-1))
objects=[]
def add(name,shape,color,group,reference=False): objects.append(dict(name=name,shape=shape,color=color,group=group,reference=reference))
add('V3_CHASSIS_300x200x4',deck,'#a5b59a','chassis')
add('PRINT_ADAPTER',adapter,'#e58c39','adapter')
add('PCB_90x70_REFERENCE',pcb,'#287b67','pcb',True)
battery=box(120,50,28,z=-38)
add('BATTERY_120x50x28_REFERENCE',battery,'#72808d','battery',True)
def nut(x,y,z): return cq.Workplane('XY').center(x,y).polygon(6,5.5/math.cos(math.pi/6)).extrude(2.4).val().translate((0,0,z)).cut(cyl(3.2,4,x,y,z-1))
def washer(x,y,z,od=7): return cyl(od,.5,x,y,z).cut(cyl(3.2,2,x,y,z-.5))
features=json.loads((BASE/'chassis_fr4_v3_300x200/output/hole_features.json').read_text(encoding='utf-8'))
for i,(x,y) in enumerate(mounts,1):
    gx,gy=cx+x,cy+y
    assert any(abs(f['x']-gx)<1e-6 and abs(f['y']-gy)<1e-6 and f['group']=='BOARD_M3' for f in features)
    assert deck.intersect(cyl(3,6,gx,gy,-5)).Volume()<1e-6
    add(f'PRINT_SPACER_{i}',spacer.translate((gx,gy,upper)),'#e58c39','spacer')
    add(f'M3x25_CHASSIS_BOLT_{i}',cyl(3,25,gx,gy,-24.5).fuse(cyl(5.5,3,gx,gy,.5)),'#b9c5cb','hardware')
    add(f'CHASSIS_TOP_WASHER_{i}',washer(gx,gy,0),'#9daab3','hardware')
    add(f'CHASSIS_BOTTOM_WASHER_{i}',washer(gx,gy,adapter_offset-.5),'#9daab3','hardware')
    add(f'CHASSIS_M3_NUT_{i}',nut(gx,gy,adapter_offset-2.9),'#b9c5cb','hardware')
for i,(x,y) in enumerate(pcbholes,1):
    gx,gy=cx+x,cy+y; headtop=pcbtop-pcbth-.5
    add(f'PCB_M3x16_BOLT_{i}',cyl(3,16,gx,gy,headtop).fuse(cyl(5.5,3,gx,gy,headtop-3)),'#b9c5cb','hardware')
    add(f'PCB_SMALL_WASHER_{i}',washer(gx,gy,headtop,5.5),'#9daab3','hardware')
    add(f'PCB_M3_NUT_{i}',nut(gx,gy,upper),'#b9c5cb','hardware')

# Check all represented bodies, including fasteners, for positive-volume interference.
overlaps=[]
for i,a in enumerate(objects):
    assert a['shape'].isValid(),a['name']
    for b in objects[i+1:]:
        ba,bb=a['shape'].BoundingBox(),b['shape'].BoundingBox()
        if any(min(getattr(ba,k+'max'),getattr(bb,k+'max'))-max(getattr(ba,k+'min'),getattr(bb,k+'min'))<1e-5 for k in 'xyz'): continue
        vol=a['shape'].intersect(b['shape']).Volume()
        if vol>1e-5: overlaps.append([a['name'],b['name'],round(vol,6)])
assert not overlaps,overlaps
assert abs(pcb.BoundingBox().xlen-70)<1e-6 and abs(pcb.BoundingBox().ylen-90)<1e-6
assert adapter.distance(battery)>=9.999
for x,y in pcbholes+mounts: assert plate.intersect(cyl(3,t+bh+2,x,y,-1)).Volume()<1e-6
assembly=cq.Assembly(name='PCB_BOTTOM_ADAPTER_V1')
for o in objects: assembly.add(o['shape'],name=o['name'],color=cq.Color(*[int(o['color'][i:i+2],16)/255 for i in (1,3,5)]))
assembly.save(str(OUT/'step/03_chassis_adapter_pcb_assembly.step'))
reread=cq.importers.importStep(str(OUT/'step/03_chassis_adapter_pcb_assembly.step')).val()
assert reread.isValid() and len(reread.Solids())==len(objects)
checks.update({'chassis_existing_holes_match':True,'all_modeled_body_interferences':overlaps,'assembly_step_solids':len(objects),'adapter_to_battery_reference_gap_mm':round(adapter.distance(battery),3),'pcb_top_z_mm':pcbtop,'pcb_bottom_z_reference_mm':pcbtop-pcbth,'pcb_rear_gap_mm':bh,'physical_fit_tested':False,'unknowns':['actual PCB component and connector envelopes','actual PCB thickness (modeled 1.6 mm)','wheelset body intrusion and ground clearance']})
(OUT/'verification.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2),encoding='utf-8')
with (OUT/'hole_coordinates_mm.csv').open('w',encoding='utf-8-sig',newline='') as f:
    wr=csv.writer(f); wr.writerow(['group','local_X_mm','local_Y_mm','hole_diameter_mm','chassis_X_mm','chassis_Y_mm'])
    for group,pts in [('PCB_64X84',pcbholes),('CHASSIS_40X100',mounts)]:
        for x,y in pts: wr.writerow([group,x,y,hole,cx+x,cy+y])

# CAD DXF is a flat-profile reference: bosses must be taken from STEP, not DXF.
doc=ezdxf.new('R2010'); doc.units=ezdxf.units.MM; ms=doc.modelspace()
for layer in ['OUTER','WINDOW','M3_HOLES','BOSS_REFERENCE']: doc.layers.new(layer)
def outline(ww,hh,r,layer):
    a,b=ww/2,hh/2;q=math.tan(math.pi/8)
    ms.add_lwpolyline([(-a+r,-b,0),(a-r,-b,q),(a,-b+r,0),(a,b-r,q),(a-r,b,0),(-a+r,b,q),(-a,b-r,0),(-a,-b+r,q)],format='xyb',close=True,dxfattribs={'layer':layer})
outline(w,h,5,'OUTER');outline(*wh,5,'WINDOW')
for x,y in pcbholes+mounts: ms.add_circle((x,y),hole/2,dxfattribs={'layer':'M3_HOLES'})
for x,y in pcbholes: ms.add_circle((x,y),2.75,dxfattribs={'layer':'BOSS_REFERENCE'})
doc.saveas(OUT/'adapter_profile_reference.dxf')
assert not ezdxf.readfile(OUT/'adapter_profile_reference.dxf').audit().has_errors

scene=[]
for o in objects:
    verts,faces=o['shape'].tessellate(.15,.18)
    scene.append({k:o[k] for k in ['name','color','group','reference']}|{'v':[[round(v.x,4),round(v.y,4),round(v.z,4)] for v in verts],'f':faces})
(OUT/'preview/scene.json').write_text(json.dumps(scene,separators=(',',':')),encoding='utf-8')
# Reuse the local orthographic depth-buffer renderer, with no external dependencies.
sys.path.insert(0,str(BASE/'vehicle_assembly_v1'))
from render_support import raster
fontpath='C:/Windows/Fonts/msyh.ttc';font=ImageFont.truetype(fontpath,26);small=ImageFont.truetype(fontpath,20)
local=[]
for name,shape,color in [('adapter',plate,'#e58c39')]:
    v,f=shape.tessellate(.12,.16);local.append({'v':[a.toTuple() for a in v],'f':f,'color':color})
canvas=Image.new('RGB',(1600,1060),'#f5f7fa');d=ImageDraw.Draw(canvas)
d.text((38,20),'小车底部电路板转接板 · 84 × 64 / M3',font=font,fill='#203b47')
canvas.paste(Image.fromarray(raster(local,760,650,40,-50)),(25,90))
d.text((45,765),'打印件：80 × 116 × 4 mm；四个 6 mm 一体支柱',font=small,fill='#203b47')
canvas.paste(Image.fromarray(raster(scene,760,650,-35,-40)),(810,90))
d.text((825,765),'底面装配：橙色转接板 / 绿色电路板 / 灰色电池占位',font=small,fill='#203b47')
for yy,txt in enumerate(['电路板 90 × 70 mm，旋转后沿车长 70、沿车宽 90；孔距沿车长 64、沿车宽 84。','利用底盘现有孔：X=90/130，Y=±50 mm；底盘与转接板间用 10 mm 隔柱。','PCB 板背间隙 6 mm；板厚暂按 1.6 mm。元件高度、插头、轮组与离地空间需按实物核对。']): d.text((45,850+yy*54),txt,font=small,fill='#4d6570')
canvas.save(OUT/'preview/adapter_and_assembly.png')

# Dimensioned SVG in millimetres (print with scale 100% only).
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="210mm" height="240mm" viewBox="-90 -110 210 240"><rect x="-90" y="-110" width="210" height="240" fill="white"/><g font-family="Microsoft YaHei" font-size="4" stroke-width="0.35">','<text x="-75" y="-94" fill="#203b47" font-size="6">底部转接板 V1 · 单位 mm</text>',f'<rect x="{-w/2}" y="{-h/2}" width="{w}" height="{h}" rx="5" fill="#fff2df" stroke="#915323"/>',f'<rect x="{-wh[0]/2}" y="{-wh[1]/2}" width="{wh[0]}" height="{wh[1]}" rx="5" fill="white" stroke="#915323"/>','<rect x="-35" y="-45" width="70" height="90" fill="none" stroke="#25806a" stroke-dasharray="2 1"/>']
for group,pts,color in [('PCB',pcbholes,'#25806a'),('车架',mounts,'#3f6db4')]:
    for x,y in pts:
        svg.append(f'<circle cx="{x}" cy="{-y}" r="1.7" fill="white" stroke="{color}"/>')
        if group=='PCB': svg.append(f'<circle cx="{x}" cy="{-y}" r="2.75" fill="none" stroke="{color}"/><circle cx="{x}" cy="{-y}" r="4.5" fill="none" stroke="{color}" stroke-dasharray="1 1"/>')
        svg.append(f'<path d="M{x-3},{-y}h6 M{x},{-y-3}v6" fill="none" stroke="{color}" stroke-width="0.15"/>')
def dim(x1,y1,x2,y2,label,tx,ty):
    svg.append(f'<path d="M{x1},{y1}L{x2},{y2}" stroke="#657987"/><path d="M{x1-1.5},{y1-1.5}l3,3 M{x2-1.5},{y2-1.5}l3,3" stroke="#657987"/><text x="{tx}" y="{ty}" fill="#203b47">{label}</text>')
dim(-40,-72,40,-72,'80',-3,-74);dim(54,-58,54,58,'116',56,2)
dim(-32,67,32,67,'64 PCB',-8,72);dim(-52,-42,-52,42,'84 PCB',-76,2)
dim(-20,80,20,80,'40 车架',-9,85);dim(76,-50,76,50,'100 车架',78,2)
svg.extend(['<text x="-75" y="98" fill="#25806a">绿：PCB 4×Ø3.4，支柱 Ø5.5 / 根部 Ø9，高 6</text>','<text x="-75" y="106" fill="#3f6db4">蓝：车架 4×Ø3.4，配 Ø8×10 隔柱</text>','<text x="-75" y="114" fill="#203b47">板厚 4；外角 R5；中窗 44×64 R5；原点为转接板中心</text>','</g></svg>'])
s='\n'.join(svg);(OUT/'preview/dimensions.svg').write_text(s,encoding='utf-8')
(OUT/'preview/dimensions.png').write_bytes(resvg_py.svg_to_bytes(svg_string=s.replace('width="210mm" height="240mm"','width="1050" height="1200"'),font_files=[fontpath]))
print(json.dumps(checks,ensure_ascii=False))
