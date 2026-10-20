"""Read validated CAD, assemble whole vehicle, and add marked reference hardware."""
from pathlib import Path
import json,math,csv
import cadquery as cq
import numpy as np
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output';BASE=ROOT.parent;V=cq.Vector;H=320

def box(x,y,z,cx=0,cy=0,z0=0):return cq.Solid.makeBox(x,y,z,V(cx-x/2,cy-y/2,z0))
def cylinder(d,h,x=0,y=0,z=0,axis=(0,0,1)):return cq.Solid.makeCylinder(d/2,h,V(x,y,z),V(*axis))
def bbarray(s):
    b=s.BoundingBox();return np.array([b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax])
def load_named(step,records):
    solids=cq.importers.importStep(str(step)).val().Solids();remaining=set(range(len(solids)));output=[]
    bounds=[bbarray(s) for s in solids]
    for rec in records:
        v=np.array(rec['v']);bb=np.r_[v.min(0),v.max(0)]
        candidates=sorted((float(np.max(np.abs(bounds[i]-bb))),i) for i in remaining)
        error,idx=candidates[0]
        assert error<.12,(rec['name'],error)
        assert len(candidates)==1 or candidates[1][0]>.12,('ambiguous STEP match',rec['name'])
        remaining.remove(idx);output.append({k:rec[k] for k in ['name','color','group','reference']}|{'shape':solids[idx],'source':'existing_CAD'})
    return output,[solids[i] for i in remaining]

v3=json.loads((BASE/'pendulum_v3/output/preview/scene.json').read_text(encoding='utf-8'))
pend,left=load_named(BASE/'pendulum_v3/output/assembly.step',v3);assert not left
pend=[o for o in pend if o['name']!='CHASSIS_DECK_ENVELOPE_ONLY']
cr=json.loads((BASE/'chassis_fr4_v1/output/preview/chassis_scene.json').read_text(encoding='utf-8'))
cr=[o for o in cr if not o['name'].startswith('ref_370')]
chassis,left=load_named(BASE/'chassis_fr4_v1/output/chassis_assembly_520_reference.step',cr);assert not left
# Replace generic solid component blocks with clearer component envelopes below.
chassis=[o for o in chassis if not o['name'].startswith('ref_board_components_')]
wr=json.loads((BASE/'pendulum_rod_weight/output/preview/accessory_scene.json').read_text(encoding='utf-8'))
weight,left=load_named(BASE/'pendulum_rod_weight/output/weight_assembly.step',wr)
assert len(left)==1 and abs(left[0].BoundingBox().zlen-260)<1e-5
scene=pend+chassis+weight

def add(name,s,color='#b8c2ca',group='hardware',reference=False,source='simplified_fastener'):
    scene.append({'name':name,'shape':s,'color':color,'group':group,'reference':reference,'source':source})
def oriented(s,pos,axis):
    target=V(*axis);z=V(0,0,1);cross=z.cross(target)
    if cross.Length>1e-8:s=s.rotate((0,0,0),cross.toTuple(),math.degrees(math.acos(z.dot(target))))
    elif target.z<0:s=s.rotate((0,0,0),(1,0,0),180)
    return s.translate(pos)
def ring(d,di,t,pos,axis=(0,0,1)):
    s=cylinder(d,t).cut(cylinder(di,t+2,z=-1));return oriented(s,pos,axis)
def hexnut(d,af,t,pos,axis=(0,0,1)):
    # Rotate the hexagon so across-flats sides align with the CAD nut traps.
    r=af/math.sqrt(3);pts=[(r*math.cos(k*math.pi/3),r*math.sin(k*math.pi/3)) for k in range(6)]
    s=cq.Workplane('XY').polyline(pts).close().extrude(t).val().cut(cylinder(d+.2,t+2,z=-1))
    return oriented(s,pos,axis)
def bolt(d,l,hd,hh,underhead,axis):
    s=cylinder(d,l).fuse(cylinder(hd,hh,z=-hh));return oriented(s,underhead,axis)
def nut_x(x,y,z,af=5.5,d=3,t=2.4):
    r=af/math.sqrt(3);pts=[(r*math.cos(i*math.pi/3),r*math.sin(i*math.pi/3)) for i in range(6)]
    plane=cq.Plane(origin=(x,y,z),xDir=(0,1,0),normal=(1,0,0))
    return cq.Workplane(plane).polyline(pts).close().extrude(t).val().cut(cylinder(d+.2,t+2,x-1,y,z,(1,0,0)))

# Existing pendulum feet: M5x20, washer top/bottom, hex nut below FR4.
for x in [-55,55]:
    for y in [-88,-42,42,88]:
        tag=f'{x}_{y}'
        add('base_M5bolt_'+tag,bolt(5,20,8.5,5,(x,y,9),(0,0,-1)))
        add('base_M5washer_top_'+tag,ring(10,5.3,1,(x,y,8)))
        add('base_M5washer_bottom_'+tag,ring(10,5.3,1,(x,y,-5)))
        add('base_M5nut_'+tag,hexnut(5,8,4,(x,y,-9)))
# Tower splice fasteners, heads outward on both sides.
for side in [-1,1]:
    for x in [-16,16]:
        for z in [155,195]:
            tag=f'{side}_{x}_{z}'
            add('splice_M4bolt_'+tag,bolt(4,35,7,4,(x,side*78,z),(0,-side,0)))
            for yy in [77,52]:add('splice_M4washer_'+tag+'_'+str(yy),ring(9,4.3,1,(x,side*yy,z),(0,side,0)))
            add('splice_M4nut_'+tag,hexnut(4,7,3.2,(x,side*48.8,z),(0,side,0)))
# Bearing covers and shared encoder rail attachment screws.
for side in [-1,1]:
    for x in [-23,23]:
        for z in [305,335]:
            ext=80 if side==1 and z==305 else 74
            pos=ext+1;length=30 if ext==80 else 25;tag=f'{side}_{x}_{z}'
            add('cover_M3bolt_'+tag,bolt(3,length,5.5,3,(x,side*pos,z),(0,-side,0)))
            add('cover_M3washer_'+tag,ring(7,3.3,1,(x,side*ext,z),(0,side,0)))
            # Pocket at the inner face, rotate nut about Y to match the trap flats.
            n=hexnut(3,5.5,2.4,(x,side*59,z),(0,side,0))
            add('cover_M3nut_'+tag,n)
# Bottom M4 tie rods already exist. Add nuts and washers at their exposed ends.
for x in [-38,38]:
    for side in [-1,1]:
        add(f'tierod_washer_{x}_{side}',ring(9,4.3,1,(x,side*80,20),(0,side,0)))
        add(f'tierod_nut_{x}_{side}',hexnut(4,7,3.2,(x,side*81,20),(0,side,0)))
# 520 bracket to main plate: M3x12, all sixteen confirmed holes.
for x in [-133,-103,103,133]:
    for y in [-102,-78,78,102]:
        tag=f'{x}_{y}'
        add('motor_mount_M3bolt_'+tag,bolt(3,12,5.5,3,(x,y,.5),(0,0,-1)))
        add('motor_mount_washer_top_'+tag,ring(7,3.3,.5,(x,y,0)))
        add('motor_mount_washer_bottom_'+tag,ring(7,3.3,.5,(x,y,-6)))
        add('motor_mount_M3nut_'+tag,hexnut(3,5.5,2.4,(x,y,-8.4)))
# Two generic board placeholders on their real universal-grid supports.
for x in [-130,-90,90,130]:
    for y in [-50,50]:
        tag=f'{x}_{y}'
        add('board_M3bolt_'+tag,bolt(3,25,5.5,3,(x,y,17.1),(0,0,-1)))
        add('board_washer_top_'+tag,ring(7,3.3,.5,(x,y,16.6)))
        add('board_washer_bottom_'+tag,ring(7,3.3,.5,(x,y,-4.5)))
        add('board_M3nut_'+tag,hexnut(3,5.5,2.4,(x,y,-6.9)))

# Pivot/rod clamp fasteners rotate together with the pendulum.
add('rotor_hub_vertical_bolt',bolt(3,30,5.5,3,(18,0,335),(0,0,-1)),group='rotor')
add('rotor_hub_vertical_washer',ring(7,3.3,1,(18,0,334)),group='rotor')
add('rotor_hub_vertical_nut',hexnut(3,5.5,2.4,(18,0,306.2)),group='rotor')
add('rotor_tube_clamp_bolt',bolt(3,30,5.5,3,(13,10,360),(-1,0,0)),group='rotor')
add('rotor_tube_clamp_washer',ring(7,3.3,1,(12,10,360),(1,0,0)),group='rotor')
add('rotor_tube_clamp_nut',nut_x(-11.9,10,360),group='rotor')
add('rotor_magnet_setscrew',bolt(3,12,5.5,3,(16,97,320),(-1,0,0)),group='rotor')
add('rotor_magnet_nut',nut_x(12,97,320),group='rotor')
# Encoder carriage and four module fasteners.
for x in [-23,23]:
    add('encoder_slide_bolt_'+str(x),bolt(3,20,5.5,3,(x,112.9,305),(0,0,-1)))
    add('encoder_slide_topwasher_'+str(x),ring(7,3.3,1,(x,112.9,304)))
    add('encoder_slide_bottomwasher_'+str(x),ring(7,3.3,1,(x,112.9,290)))
    add('encoder_slide_nut_'+str(x),hexnut(3,5.5,2.4,(x,112.9,287.6)))
for x in [-8.5,8.5]:
    for z in [311.5,328.5]:
        tag=f'{x}_{z}'
        add('encoder_PCB_bolt_'+tag,bolt(2.5,16,4.5,1.7,(x,112.8,z),(0,1,0)))
        add('encoder_PCB_frontwasher_'+tag,ring(6,2.7,.5,(x,112.8,z),(0,1,0)))
        add('encoder_PCB_backwasher_'+tag,ring(6,2.7,.5,(x,124.9,z),(0,1,0)))
        add('encoder_PCB_nut_'+tag,hexnut(2.5,5,2,(x,125.4,z),(0,1,0)))

# All electronics in this section are layout envelopes, not a G407 or DRV8871 PCB CAD.
def elec(name,s,color):add('layout_'+name,s,color,'reference',True,'unverified_electronics_envelope')
elec('main_MCU',box(14,14,2,cx=105,z0=16.6),'#25313b')
elec('USB',box(10,8,5,cx=140,z0=16.6),'#bac1c4')
elec('power_terminal',box(10,18,10,cx=80,cy=35,z0=16.6),'#508a50')
for y in [-63,63]:elec('header_'+str(y),box(36,4,7,cx=110,cy=y,z0=16.6),'#323e49')
for x in [-125,-95]:
    for y in [-25,25]:
        key=f'{x}_{y}'
        elec('driver_insulation_'+key,box(22,24,2,x,y,16.6),'#dad2bb')
        elec('DRV8871_module_PCB_'+key,box(22,24,1.6,x,y,18.6),'#376b72')
        elec('driver_IC_'+key,box(5,5,1,x-1,y,20.2),'#23313b')
        elec('driver_terminal_'+key,box(8,10,7,x+6,y,20.2),'#4d8155')
        elec('driver_capacitor_'+key,cylinder(5,7,x-6,y-5,20.2),'#404a51')
# Small generic IMU on the center's 20x20 mounting pattern, height <15.
imu=box(30,30,1.6,z0=5.5)
for x in [-10,10]:
    for y in [-10,10]:
        imu=imu.cut(cylinder(3.4,4,x,y,4.5))
        elec(f'IMU_standoff_{x}_{y}',ring(5,3.3,5.5,(x,y,0)),'#dad4c4')
        add(f'IMU_mount_bolt_{x}_{y}',bolt(3,16,5.5,3,(x,y,7.6),(0,0,-1)))
        add(f'IMU_mount_topwasher_{x}_{y}',ring(7,3.3,.5,(x,y,7.1)))
        add(f'IMU_mount_bottomwasher_{x}_{y}',ring(7,3.3,.5,(x,y,-4.5)))
        add(f'IMU_mount_nut_{x}_{y}',hexnut(3,5.5,2.4,(x,y,-6.9)))
elec('IMU_PCB',imu,'#427d83');elec('IMU_chip',box(4,4,1.3,z0=7.1),'#25313b')
elec('IMU_header',box(7,3,5,cy=13,z0=7.1),'#303c46')
# Pads and two strap loops pass through the real X=+/-40 slots.
for x in [-45,45]:add('battery_pad_top_'+str(x),box(18,50,6,x,0,-10),'#6b7278','reference',True,'battery_mount_reference')
add('battery_pad_bottom',box(120,50,1,z0=-39),'#6b7278','reference',True,'battery_mount_reference')
for x in [-40,40]:
    outer=box(12,62,41,x,0,-40);inner=box(14,60,39,x,0,-39)
    add('battery_strap_'+str(x),outer.cut(inner).clean(),'#414c58','reference',True,'battery_mount_reference')

def route(points,d):
    chunks=[]
    for a,b in zip(points,points[1:]):
        delta=V(*b)-V(*a);chunks.append(cq.Solid.makeCylinder(d/2,delta.Length,V(*a),delta.normalized()))
    for p in points[1:-1]:chunks.append(cq.Solid.makeSphere(d/2,V(*p)))
    s=chunks[0]
    for c in chunks[1:]:s=s.fuse(c)
    return s.clean()
add('route_encoder',route([(0,124,307),(0,136,305),(32,136,292),(40,90,280),(40,90,40),(90,85,30),(115,63,24)],2.5),'#d79d45','routing',True,'illustrative_not_wiring_pinout')
add('route_power',route([(60,15,-18),(65,27,-8),(0,27,-8),(0,20,-8),(0,20,3),(70,20,4),(80,35,24)],2.5),'#ac4f4c','routing',True,'illustrative_not_wiring_pinout')

# Full physical-body overlap test. Routes can terminate inside connector envelopes
# and are explicitly not modeled cable/housing detail.
physical=[o for o in scene if o['group']!='routing']
bounds=[bbarray(o['shape']) for o in physical];hits=[]
for i,a in enumerate(physical):
    assert a['shape'].isValid(),a['name']
    for j,b in enumerate(physical[i+1:],i+1):
        ba,bb=bounds[i],bounds[j]
        if np.any(ba[3:]<=bb[:3]+1e-5) or np.any(bb[3:]<=ba[:3]+1e-5):continue
        volume=a['shape'].intersect(b['shape']).Volume()
        if volume>.015:hits.append({'a':a['name'],'b':b['name'],'volume':round(volume,5)})
(OUT/'collision_report.json').write_text(json.dumps(hits,ensure_ascii=False,indent=2),encoding='utf-8')
print('INTERFERENCE',json.dumps(hits,ensure_ascii=False),flush=True)
assert not hits,'Resolve assembly overlaps before delivery'

# Continuous conservative motion envelopes for the new rotating fasteners too.
central=[o for o in scene if o['group']=='rotor' and (o['name'].startswith(('weight_','tube_','rotor_hub_','rotor_tube_')) or o['name']=='rod_hub')]
rad=0;cymin=1e9;cymax=-1e9
for o in central:
    b=bbarray(o['shape']);rad=max(rad,math.hypot(max(abs(b[0]),abs(b[3])),max(abs(b[2]-H),abs(b[5]-H))));cymin=min(cymin,b[1]);cymax=max(cymax,b[4])
min_z=H-rad
fixed=[o for o in scene if o['group'] not in ['rotor','routing']]
for o in fixed:
    b=bbarray(o['shape'])
    assert b[4]<=cymin or b[1]>=cymax or b[5]<min_z,('central sweep requires extra check',o['name'])
mag=[o for o in scene if o['name'].startswith(('magnet_cup','magnet_6x3','rotor_magnet_'))]
mb=[bbarray(o['shape']) for o in mag];mr=max(math.hypot(max(abs(b[0]),abs(b[3])),max(abs(b[2]-H),abs(b[5]-H))) for b in mb)
my0=min(b[1] for b in mb);my1=max(b[4] for b in mb)
menv=cylinder(2*mr,my1-my0,0,my0,H,(0,1,0))
for o in fixed:
    b=bbarray(o['shape'])
    if b[4]<my0 or b[1]>my1 or b[5]<H-mr or b[2]>H+mr:continue
    assert menv.intersect(o['shape']).Volume()<.015,('magnet sweep collision',o['name'])

assembly=cq.Assembly(name='Complete_Pendulum_Car_V1_520')
records=[]
for o in scene:
    rgb=[int(o['color'][i:i+2],16)/255 for i in (1,3,5)]
    assembly.add(o['shape'],name=o['name'],color=cq.Color(*rgb))
    vv,ff=o['shape'].tessellate(.2,.15)
    records.append({k:o[k] for k in ['name','color','group','reference','source']}|{'v':[[round(a,4) for a in v.toTuple()] for v in vv],'f':[list(f) for f in ff]})
assembly.save(str(OUT/'vehicle_complete_520.step'))
step=cq.importers.importStep(str(OUT/'vehicle_complete_520.step')).val();assert step.isValid()
count=len(step.Solids());assert count==sum(len(o['shape'].Solids()) for o in scene)
allbb=np.array([bbarray(o['shape']) for o in scene]);extent=allbb[:,3:].max(0)-allbb[:,:3].min(0)
report={'components':len(scene),'step_solids':count,'step_valid':True,'physical_body_overlap_pairs':len(hits),'central_continuous_sweep_min_z_mm':round(min_z,3),'magnet_fastener_full_rotation_envelope_radius_mm':round(mr,3),'rotating_fastener_motion_envelopes_clear':True,'reference_envelope_xyz_mm':np.round(extent,2).tolist(),'ground_at_z_mm':-63.5,'rod_tip_z_mm':600,'ballast_lower_edge_from_pivot_mm':160,'ballast_washers':2,'unverified':['motor output-axis offset and shaft length','wheel hub and tread','G407/DRV8871 PCB actual shapes and mounting patterns','battery case and plugs','routing and real cable lengths','physical strength and control performance']}
(OUT/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'preview/scene.json').write_text(json.dumps(records,separators=(',',':')),encoding='utf-8')
with (OUT/'component_manifest.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['component','category','source','reference'])
    for o in scene:w.writerow([o['name'],o['group'],o['source'],o['reference']])
print('COMPLETE',json.dumps(report),flush=True)
import draw_vehicle
draw_vehicle.render(ROOT,OUT,records,report)
