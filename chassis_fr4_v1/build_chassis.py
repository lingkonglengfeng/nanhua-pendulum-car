"""FR4 chassis for existing V3 pendulum. Generates STEP, DXF, drawings and hole table."""
from pathlib import Path
import math,json,csv,hashlib
import cadquery as cq
import numpy as np
import trimesh,ezdxf

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output';P=json.loads((ROOT/'parameters.json').read_text(encoding='utf-8'))
for folder in ['step','dxf','stl_reference','preview']:(OUT/folder).mkdir(parents=True,exist_ok=True)
V=cq.Vector
def box(dx,dy,dz,x=0,y=0,z=0):return cq.Solid.makeBox(dx,dy,dz,V(x-dx/2,y-dy/2,z))
def cylinder(d,h,x=0,y=0,z=0,axis=(0,0,1)):return cq.Solid.makeCylinder(d/2,h,V(x,y,z),V(*axis))
def cy(d,y0,h,x,z):return cylinder(d,h,x,y0,z,(0,1,0))
def slot_shape(x,y,length,width,angle,z,h):
    a=(length-width)/2;s=box(2*a,width,h,z=z).fuse(cylinder(width,h,-a,0,z),cylinder(width,h,a,0,z))
    return s.rotate((0,0,0),(0,0,1),angle).translate((x,y,0))
def rounded_plate(w,h,t,z=0,r=3):return cq.Workplane(obj=box(w,h,t,z=z)).edges('|Z').fillet(r).val()

holes=[]
def hole(group,x,y,d):holes.append({'group':group,'kind':'circle','x':x,'y':y,'diameter':d})
def slot(group,x,y,l,w,a):holes.append({'group':group,'kind':'slot','x':x,'y':y,'length':l,'width':w,'angle':a})
for x in P['pendulum_base_holes_x']:
    for y in P['pendulum_base_holes_y']:hole('BASE_M5',x,y,5.5)
for sx in [-1,1]:
    for sy in [-1,1]:
        for dx in [-28,28]:
            for dy in [-18,18]:slot('MOTOR_M4',sx*118+dx,sy*80+dy,10,4.5,90)
        for dx in [-15,15]:
            for dy in [-12,12]:hole('MOTOR520_M3',sx*118+dx,sy*(90+dy),3.4)
for sx in [-1,1]:
    for x in [90,110,130]:
        for y in [-50,-30,-10,10,30,50]:hole('BOARD_M3',sx*x,y,3.4)
for x in [-10,10]:
    for y in [-10,10]:hole('IMU_M3',x,y,3.4)
for sx in [-1,1]:
    for y in [-30,0,30]:slot('SENSOR_M3',sx*150,y,18,3.4,90)
for x in [-40,40]:
    for y in [-30,30]:slot('BATTERY_STRAP',x,y,18,3.5,0)
for y in [-20,20]:slot('WIRE',0,y,16,4,0)
for x in [-60,-20,20,60]:
    for y in [-102,102]:hole('SIDE_M3',x,y,3.4)
for i,h in enumerate(holes,1):h['id']=f'H{i:03d}'

def cut_for(h,z=-10,depth=20):
    if h['kind']=='circle':return cylinder(h['diameter'],depth,h['x'],h['y'],z)
    return slot_shape(h['x'],h['y'],h['length'],h['width'],h['angle'],z,depth)
raw=rounded_plate(P['plate_length_x'],P['plate_width_y'],P['plate_thickness'],-P['plate_thickness'],P['outer_corner_radius'])
plate=raw
cutters=[]
for h in holes:
    c=cut_for(h);cutters.append(c);plate=plate.cut(c)
plate=plate.clean()
assert plate.isValid() and len(plate.Solids())==1
assert min(raw.intersect(c).Volume() for c in cutters)>0
for i,a in enumerate(cutters):
    for j,b in enumerate(cutters[i+1:],i+1):assert a.intersect(b).Volume()<1e-5,(holes[i]['id'],holes[j]['id'])

adapter_holes=[]
for x in [-28,28]:
    for y in [-18,18]:adapter_holes.append({'group':'ADAPTER_M4','kind':'circle','x':x,'y':y,'diameter':4.5})
blank=rounded_plate(72,60,3,r=3)
for h in adapter_holes:blank=blank.cut(cut_for(h))
blank=blank.clean()
adjust_holes=adapter_holes+[
    {'group':'BRACKET_M3','kind':'slot','x':x,'y':8,'length':36,'width':7.4,'angle':90} for x in [-13.5,13.5]
]
adjust=blank
for h in adjust_holes[4:]:adjust=adjust.cut(cut_for(h))
adjust=adjust.clean()
parts=[('01_chassis_FR4_320x220x4',plate,1,320,220,4,10,holes),('02_motor_adapter_adjustable_72x60x3',adjust,4,72,60,3,3,adjust_holes),('03_motor_adapter_blank_72x60x3',blank,0,72,60,3,3,adapter_holes)]

def rounded_outline(ms,w,h,r):
    a,b=w/2,h/2;q=math.tan(math.pi/8)
    pts=[(-a+r,-b,0),(a-r,-b,q),(a,-b+r,0),(a,b-r,q),(a-r,b,0),(-a+r,b,q),(-a,b-r,0),(-a,-b+r,q)]
    ms.add_lwpolyline(pts,format='xyb',close=True,dxfattribs={'layer':'CUT_OUTLINE'})
def dxf_file(name,w,h,r,features):
    doc=ezdxf.new('R2010');doc.units=ezdxf.units.MM;doc.header['$MEASUREMENT']=1
    for layer in ['CUT_OUTLINE']+sorted(set('CUT_'+a['group'] for a in features)):doc.layers.new(layer)
    ms=doc.modelspace();rounded_outline(ms,w,h,r)
    for f in features:
        layer='CUT_'+f['group'];x,y=f['x'],f['y']
        if f['kind']=='circle':ms.add_circle((x,y),f['diameter']/2,dxfattribs={'layer':layer})
        else:
            rr=f['width']/2;a=(f['length']-f['width'])/2;t=math.radians(f['angle'])
            pts=[]
            for u,v,b in [(-a,-rr,0),(a,-rr,1),(a,rr,0),(-a,rr,1)]:pts.append((x+u*math.cos(t)-v*math.sin(t),y+u*math.sin(t)+v*math.cos(t),b))
            ms.add_lwpolyline(pts,format='xyb',close=True,dxfattribs={'layer':layer})
    path=OUT/'dxf'/f'{name}.dxf';doc.saveas(path)
    read=ezdxf.readfile(path);assert len(read.modelspace())==1+len(features);assert not read.audit().has_errors

report={'revision':P['revision'],'parts':[],'checks':{},'limitations':['No physical machining, load or vibration validation','520 bracket follows user drawing; 370 bracket and both motor body/shaft dimensions remain unverified','Controller board and battery placeholders are not a verified product fit']}
for name,s,qty,w,h,t,r,features in parts:
    assert s.isValid() and len(s.Solids())==1
    cq.exporters.export(s,str(OUT/'step'/f'{name}.step'))
    b=s.BoundingBox();ps=s.translate((-b.xmin,-b.ymin,-b.zmin))
    cq.exporters.export(ps,str(OUT/'stl_reference'/f'{name}.stl'),tolerance=.06,angularTolerance=.12)
    mesh=trimesh.load_mesh(OUT/'stl_reference'/f'{name}.stl',process=True);assert mesh.is_volume and mesh.is_watertight
    dxf_file(name,w,h,r,features)
    report['parts'].append({'id':name,'quantity':qty,'blank_optional':qty==0,'size':[w,h,t],'volume_cm3':round(s.Volume()/1000,3),'FR4_mass_g_at_1_9':round(s.Volume()*.0019,1),'features':len(features)})

# Check V3 mounting holes against the real imported original base solids.
base=cq.importers.importStep(str(ROOT.parent/'pendulum_v3/output/step/01_base_shoe.step')).val()
for sign in [-1,1]:
    b=base.translate((0,sign*65,0));assert b.intersect(plate).Volume()<1e-6
    for x in [-55,55]:
        for yy in [-23,23]:assert b.intersect(cylinder(5,10,x,sign*65+yy,-1)).Volume()<1e-6
    # Motor interface top-side washer envelopes stay outside the 160mm long base.
    for h in holes:
        if h['group'] in ['MOTOR_M4','MOTOR520_M3']:assert b.intersect(cylinder(9,1,h['x'],h['y'],0)).Volume()<1e-6

tested=[]
for px in [24,27,30]:
    for py in [0,12,20,24,30]:
        for x in [-px/2,px/2]:
            for y in sorted(set([8-py/2,8+py/2])):
                assert adjust.intersect(cylinder(3,5,x,y,-1)).Volume()<1e-6,(px,py,x,y)
        tested.append([px,py])

scene=[]
def add(name,s,color,group='fixed',reference=False):scene.append({'name':name,'shape':s,'color':color,'group':group,'reference':reference})
add('FR4_MAIN_CHASSIS',plate,'#9bac82')
for sx in [-1,1]:
    for sy in [-1,1]:
        tag=f'{sx}_{sy}'
        def place(s):
            if sy<0:s=s.rotate((0,0,0),(0,0,1),180)
            return s.translate((sx*118,sy*80,-15))
        add('ref_370_adapter_'+tag,place(adjust),'#6e895e')
        for dx in [-28,28]:
            for dy in [-18,18]:
                x=sx*118+dx;y=sy*80+dy
                spacer=cylinder(8,8,x,y,-12).cut(cylinder(4.3,10,x,y,-13))
                add('ref_370_standoff_'+tag+f'_{dx}_{dy}',spacer,'#b9c5ce','hardware')
                # Upward head, washer and under-side nut: simplified reference geometry.
                shank=cylinder(4,25,x,y,-24).fuse(cylinder(7,4,x,y,1))
                add('ref_370_M4bolt_'+tag+f'_{dx}_{dy}',shank,'#b9c5ce','hardware')

fixed_main=[o for o in scene if o['name'].startswith(('FR4_','ref_370_adapter_','ref_370_standoff_'))]
for i,a in enumerate(fixed_main):
    for b in fixed_main[i+1:]:assert a['shape'].intersect(b['shape']).Volume()<.001,(a['name'],b['name'])
for typ,diam,axisz,width,px,py in [('520',37,-31,40,30,24),('370',25,-33.5,32,24,20)]:
    for sx in [-1,1]:
        for sy in [-1,1]:
            tag=typ+f'_{sx}_{sy}'
            def reflect(s):
                if sy<0:s=s.rotate((0,0,0),(0,0,1),180)
                return s.translate((sx*118,0,0))
            if typ=='520':
                # Exact supplied L-bracket profile, flipped beneath the main plate.
                foot=box(40,40,1.5,y=90,z=-5.5)
                foot=foot.fillet(2,[e for e in foot.Edges() if e.geomType()=='LINE' and e.BoundingBox().zlen>1 and abs(e.Center().y-70)<1e-6])
                bracket=foot.fuse(box(40,1.5,27,y=109.25,z=-31),cy(40,108.5,1.5,0,-31))
                slotcut=box(13,4,14,y=109.25,z=axisz-7).fuse(cy(13,107,4,0,axisz-7),cy(13,107,4,0,axisz+7))
                bracket=bracket.cut(slotcut)
                for angle in range(0,360,60):
                    a=math.radians(angle);bracket=bracket.cut(cy(3.1,107,4,15.5*math.cos(a),axisz+15.5*math.sin(a)))
                for xx in [-15,15]:
                    for yy in [78,102]:
                        bracket=bracket.cut(cylinder(3.4,4,xx,yy,-7))
                        assert plate.intersect(cylinder(3,6,sx*118+(xx if sy>0 else -xx),sy*yy,-5)).Volume()<1e-6
                body_front=108.5
            else:
                # 370 geometry is still only a clearance example; real bracket pending.
                flange_bottom=axisz-diam/2-3
                bracket=box(width,40,2,y=88,z=-17).fuse(box(width,2,-15-flange_bottom,y=109,z=flange_bottom))
                bracket=bracket.cut(cy(10,107,4,0,axisz))
                for xx in [-px/2,px/2]:
                    for yy in [88-py/2,88+py/2]:bracket=bracket.cut(cylinder(3.4,4,xx,yy,-18))
                body_front=108
            add('ref_'+tag+'_Lbracket',reflect(bracket.clean()),'#8c9dab','reference',True)
            motor=cy(diam,body_front-70,70,0,axisz)
            add('ref_'+tag+'_motor',reflect(motor),'#a8b3ba','reference',True)
            shaft=cy(6 if typ=='520' else 4,body_front,18,0,axisz)
            add('ref_'+tag+'_shaft',reflect(shaft),'#c3cbd0','reference',True)
            wheel=cy(65,114,26,0,axisz).cut(cy(6 if typ=='520' else 4,113,28,0,axisz))
            add('ref_'+tag+'_wheel',reflect(wheel),'#303a40','reference',True)
            # M3 shank through bracket/adapter, top nut envelope fits in 8mm gap.
            if typ=='370':
                for xx in [-px/2,px/2]:
                    for yy in [88-py/2,88+py/2]:
                        worldx=sx*118+(xx if sy>0 else -xx);worldy=sy*yy
                        nutenv=cylinder(6,3.4,worldx,worldy,-12)
                        assert nutenv.intersect(plate).Volume()<1e-6
            assert reflect(motor).intersect(plate).Volume()<1e-6
            assert reflect(wheel).intersect(plate).Volume()<1e-6
            if typ=='370':assert reflect(bracket).intersect(place(adjust)).Volume()<1e-6
            else:assert reflect(bracket).intersect(plate).Volume()<1e-6

battery=box(120,50,28,z=-38)
add('ref_battery_120x50x28',battery,'#465764','reference',True)
assert battery.intersect(plate).Volume()<1e-6
for sx in [-1,1]:
    board=box(70,140,1.6,x=sx*110,z=15)
    for xx in [sx*90,sx*130]:
        for yy in [-50,50]:
            board=board.cut(cylinder(3.4,4,xx,yy,14))
            spacer=cylinder(6,15,xx,yy,0).cut(cylinder(3.3,17,xx,yy,-1))
            add('ref_board_standoff_'+str(sx)+f'_{xx}_{yy}',spacer,'#d8d4c8','reference',True)
    add('ref_controller_board_'+str(sx),board,'#467f77','reference',True)
    # Components limited to 12mm above the reference PCB.
    add('ref_board_components_'+str(sx),box(58,118,12,x=sx*110,z=16.6),'#4c626e','reference',True)

assembly=cq.Assembly(name='FR4_Chassis_V1_520_reference')
for o in scene:
    if o['name'].startswith('ref_370'):continue
    rgb=[int(o['color'][i:i+2],16)/255 for i in [1,3,5]]
    assembly.add(o['shape'],name=o['name'],color=cq.Color(*rgb))
assembly.save(str(OUT/'chassis_assembly_520_reference.step'))
assert cq.importers.importStep(str(OUT/'chassis_assembly_520_reference.step')).val().isValid()
asm370=cq.Assembly(name='FR4_Chassis_V1_370_reference')
for o in scene:
    if o['name'].startswith('ref_520'):continue
    rgb=[int(o['color'][i:i+2],16)/255 for i in [1,3,5]]
    asm370.add(o['shape'],name=o['name'],color=cq.Color(*rgb))
asm370.save(str(OUT/'chassis_assembly_370_reference.step'))
assert cq.importers.importStep(str(OUT/'chassis_assembly_370_reference.step')).val().isValid()

def mesh(o):
    v,f=o['shape'].tessellate(.20,.15)
    return {k:o[k] for k in ['name','color','group','reference']}|{'v':[[round(a,4) for a in vv.toTuple()] for vv in v],'f':[list(ff) for ff in f]}
chassis_scene=[mesh(o) for o in scene]
existing=json.loads((ROOT.parent/'pendulum_v3/output/preview/scene.json').read_text(encoding='utf-8'))
existing=[o for o in existing if o['name']!='CHASSIS_DECK_ENVELOPE_ONLY']
full=existing+chassis_scene
(OUT/'preview/scene.json').write_text(json.dumps(full,separators=(',',':')),encoding='utf-8')
(OUT/'preview/chassis_scene.json').write_text(json.dumps(chassis_scene,separators=(',',':')),encoding='utf-8')

# Exact Y separation for high structures, and conservative top limit for electronics.
# All added upper-side electronics are below 28.6mm; V3 sweep bottom is 39.97mm.
report['checks']={'main_feature_count':len(holes),'base_holes_match_v3':True,'old_base_vs_motor_fastener_clear':True,'fixed_chassis_part_overlap':False,'adapter_tested_M3_patterns_xy':tested,'wheel_to_plate_lateral_reference_clearance_mm':4,'electronics_top_reference_mm':28.6,'pendulum_sweep_min_z_mm':39.971,'electronics_to_sweep_vertical_margin_mm':11.371,'520_reference_ground_clearance_under_battery_mm':25.5,'520_battery_ground_clearance_if_axis_offset_pm7_mm':[18.5,32.5],'battery_gap_below_plate_mm':6.0,'370_reference_ground_clearance_under_battery_mm':28.0,'machining_dxf_readback':True,'520_bracket_from_user_drawing':True,'520_actual_output_axis_height_unverified':True,'370_bracket_not_verified':True,'physical_strength_not_tested':True}
(OUT/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'hole_features.json').write_text(json.dumps(holes,ensure_ascii=False,indent=2),encoding='utf-8')
with (OUT/'hole_coordinates_mm.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['编号','用途','形状','X mm','Y mm','直径mm','总长mm','槽宽mm','角度deg'])
    for h in holes:w.writerow([h['id'],h['group'],h['kind'],h['x'],h['y'],h.get('diameter',''),h.get('length',''),h.get('width',''),h.get('angle','')])
print(json.dumps(report,ensure_ascii=False),flush=True)
import draw_chassis
draw_chassis.render(ROOT,OUT,P,holes,chassis_scene,full)
