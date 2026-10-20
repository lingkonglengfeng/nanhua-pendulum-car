"""Parametric cart-pendulum prototype. Units mm, X forward, Y shaft, Z up.
Run with CadQuery 2.5.2, trimesh and matplotlib. No printer G-code is generated.
"""
from pathlib import Path
import json, math, zipfile, csv
import cadquery as cq
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent
P = json.loads((ROOT / 'parameters.json').read_text(encoding='utf-8'))
OUT = ROOT / 'output'
for d in ['stl', 'step', '3mf', 'preview']:
    (OUT / d).mkdir(parents=True, exist_ok=True)
H = P['pivot_height_above_deck']
C = P['tower_center_spacing'] / 2
T = P['tower_thickness']
S = P['tower_split_height']
V = cq.Vector

def box(dx,dy,dz,x=0,y=0,z=0):
    return cq.Solid.makeBox(dx,dy,dz,V(x-dx/2,y-dy/2,z))

def cyl(r,h,x=0,y=0,z=0,axis=(0,0,1)):
    return cq.Solid.makeCylinder(r,h,V(x,y,z),V(*axis))

def cy(d,y0,length,x=0,z=H):
    return cyl(d/2,length,x,y0,z,(0,1,0))

def prism(points,y0,thickness):
    pl = cq.Plane(origin=(0,y0+thickness,0),xDir=(1,0,0),normal=(0,-1,0))
    return cq.Workplane(pl).polyline(points).close().extrude(thickness).val()

def hex_y(x,z,y0,depth,flats=5.8):
    r=flats/math.sqrt(3)
    pts=[(x+r*math.cos(math.pi*k/3),z+r*math.sin(math.pi*k/3)) for k in range(6)]
    return prism(pts,y0,depth)

def slot_z(x,y,z,length,width,height,along='x'):
    a=(length-width)/2
    if along=='x':
        return box(2*a,width,height,x,y,z).fuse(cyl(width/2,height,x-a,y,z),cyl(width/2,height,x+a,y,z))
    return box(width,2*a,height,x,y,z).fuse(cyl(width/2,height,x,y-a,z),cyl(width/2,height,x,y+a,z))

def finish(s):
    return s.clean()

def fillet_where(s,r,predicate,expected=None):
    edges=[e for e in s.Edges() if predicate(e)]
    if expected is not None:
        assert len(edges)==expected, f'Fillet selector expected {expected}, got {len(edges)}'
    assert edges, 'Fillet selector matched no edges'
    result=s.fillet(r,edges).clean()
    assert result.isValid(), 'Invalid fillet result'
    return result

def near(a,b): return abs(a-b)<1e-5

def along(e,axis):
    if e.geomType()!='LINE':return False
    bb=e.BoundingBox()
    return getattr(bb,axis+'len')>1e-4 and all(getattr(bb,k+'len')<1e-5 for k in 'xyz' if k!=axis)

def rounded_window(points,y0,thickness,r):
    # Fillet the cutter's extrusion edges: add material at the opening's corners,
    # rather than rounding only the visible rim of an unchanged sharp window.
    return fillet_where(prism(points,y0,thickness),r,lambda e:along(e,'y'),len(points))

def base_shoe():
    s=box(160,60,8)
    s=cq.Workplane(obj=s).edges('|Z').fillet(3).val()
    s=s.fuse(box(130,30,24,z=8))
    s=s.cut(box(116.4,T+0.4,25,z=8))
    for x in [-38,38]:
        s=s.cut(cy(4.5,-35,70,x,20))
    for x in [-55,55]:
        for y in [-23,23]:
            s=s.cut(slot_z(x,y,-1,24,5.6,10))
    return finish(s)

def lower_arm():
    s=prism([(-58,8),(58,8),(58,34),(32,S),(-32,S),(-58,34)],-T/2,T)
    # Preserve the entire z<=34 insertion tongue for the already printed V1 base.
    s=s.cut(rounded_window([(-40,45),(40,45),(23,S-47),(-23,S-47)],-T/2-1,T+2,P['lower_window_radius']))
    for x in [-38,38]: s=s.cut(cy(4.5,-T,2*T,x,20))
    for x in [-16,16]: s=s.cut(cy(4.5,-T,2*T,x,S-20))
    return finish(s)

def upper_arm():
    s=box(64,T,H+24-S,z=S)
    s=fillet_where(s,3,lambda e:along(e,'y') and near(e.Center().z,H+24),2)
    s=s.cut(rounded_window([(-16,S+49),(16,S+49),(13,H-36),(-13,H-36)],-T/2-1,T+2,P['upper_window_radius']))
    for x in [-16,16]: s=s.cut(cy(4.5,-T,2*T,x,S+20))
    s=s.cut(cy(16.6,-T,2*T))
    s=s.cut(cy(P['bearing_seat_diameter'],T/2-P['bearing_seat_depth'],P['bearing_seat_depth']+1))
    for x in [-23,23]:
        for z in [H-15,H+15]:
            s=s.cut(cy(3.4,-T,2*T,x,z)).cut(hex_y(x,z,-T/2-0.1,3.0))
    return finish(s)

def splice():
    s=box(58,6,70,z=S-35)
    s=fillet_where(s,4,lambda e:along(e,'y'),4)
    for x in [-16,16]:
        for z in [S-20,S+20]: s=s.cut(cy(4.5,-8,16,x,z))
    return finish(s)

def cover():
    s=fillet_where(box(60,3,44,z=H-22),3,lambda e:along(e,'y'),4).cut(cy(17.5,-5,10))
    for x in [-23,23]:
        for z in [H-15,H+15]: s=s.cut(cy(3.4,-5,10,x,z))
    return finish(s)

def rod_hub():
    s=box(48,32,28,z=H-14).fuse(box(24,32,40,z=H+12)).clean()
    # Includes both concave T-shoulder roots and six external profile corners.
    s=fillet_where(s,P['hub_profile_radius'],lambda e:along(e,'y'),8)
    s=s.cut(cy(P['shaft_bore_diameter'],-20,40))
    s=s.cut(box(26,36,1.2,x=13,z=H-0.6))
    s=s.cut(cyl(1.7,32,18,0,H-16))
    # M3 nut recess on lower face, coaxial with vertical pinch screw.
    pts=[(18+3.35*math.cos(k*math.pi/3),3.35*math.sin(k*math.pi/3)) for k in range(6)]
    nut=cq.Workplane('XY',origin=(0,0,H-14.1)).polyline(pts).close().extrude(3.1).val()
    s=s.cut(nut)
    s=s.cut(cyl(P['tube_bore_diameter']/2,34,0,0,H+20))
    s=s.cut(box(1.2,18,34,y=9,z=H+20))
    # Rounded slit termination below the tube socket (not a cosmetic edge radius).
    s=s.cut(cy(1.6,0,18,0,H+20))
    s=s.cut(cyl(1.7,28,-14,10,H+40,(1,0,0)))
    # Nut pocket at x=-12, with its axis along X.
    nut2=hex_y(0,H+40,0,3.2).rotate((0,0,0),(0,0,1),-90).translate((-12.1,10,0))
    s=s.cut(nut2)
    return finish(s)

def encoder_rail():
    # Two feet use the lower M3 bearing-cover holes; rail slots adjust Y distance.
    chunks=[]
    for x in [-23,23]:
        a=box(14,6,20,x,C+12,H-25)
        a=a.cut(cy(3.4,C+7,12,x,H-15))
        a=a.fuse(box(14,59,8,x,C+38.5,H-29)).clean()
        a=fillet_where(a,P['encoder_rail_root_radius'],lambda e:along(e,'x') and near(e.Center().y,C+15) and near(e.Center().z,H-21),1)
        a=a.cut(slot_z(x,C+49,H-30,30,3.4,11,along='y'))
        chunks.append(a)
    return finish(chunks[0].fuse(chunks[1],box(46,6,8,0,C+12,H-29)))

def encoder_carrier():
    # Provisional 30x30 PCB, hole pitch configurable. Back plate with chip access.
    y=C+54
    s=box(64,4,52,0,y,H-21)
    s=fillet_where(s,3,lambda e:along(e,'y') and near(e.Center().z,H+31),2)
    s=s.cut(rounded_window([(-8.5,H-8.5),(8.5,H-8.5),(8.5,H+8.5),(-8.5,H+8.5)],y-4,8,2))
    for x in [-P['pcb_hole_pitch_x']/2,P['pcb_hole_pitch_x']/2]:
        for z in [H-P['pcb_hole_pitch_z']/2,H+P['pcb_hole_pitch_z']/2]:
            s=s.cut(cy(P['pcb_mount_hole_diameter'],y-5,10,x,z))
    for x in [-23,23]:
        s=s.fuse(box(12,26,5,x,y-3,H-21)).clean()
    s=fillet_where(s,P['encoder_carrier_root_radius'],lambda e:along(e,'x') and near(e.Center().z,H-16) and (near(e.Center().y,y-2) or near(e.Center().y,y+2)),4)
    for x in [-23,23]:
        # Move vertical screws out from directly under the upright plate. This
        # preserves the old rail but gives a clear driver/head approach from above.
        s=s.cut(cyl(1.7,8,x,y-10,H-22))
    return finish(s)

def magnet_cup():
    boss=fillet_where(box(10,10,10,10,C+32,H-5),1,lambda e:along(e,'x'),4)
    s=cy(18,C+27,18).fuse(boss).clean()
    s=s.cut(cy(P['shaft_bore_diameter'],C+26,11))
    s=s.cut(cy(P['magnet_seat_diameter'],C+41.8,3.3))
    s=s.cut(cyl(1.7,14,2,C+32,H,(1,0,0)))
    # Hexagonal M3 nut pocket for the radial set screw.
    nut=hex_y(0,H,0,3.2).rotate((0,0,0),(0,0,1),-90).translate((11.9,C+32,0))
    s=s.cut(nut)
    return finish(s)

def coupon():
    s=box(130,60,9)
    for x,d in zip([-40,0,40],[22.1,22.2,22.3]):
        s=s.cut(cyl(8.3,11,x,10,-1)).cut(cyl(d/2,8,x,10,1.8))
    for x,d in zip([-45,-29,-13],[8.1,8.2,8.3]): s=s.cut(cyl(d/2,11,x,-18,-1))
    for x,d in zip([15,30,45],[6.1,6.2,6.3]): s=s.cut(cyl(d/2,11,x,-18,-1))
    return finish(s)

specs=[
 ('01_base_shoe','底座插槽座',base_shoe(),2,'upright'),
 ('02_lower_arm','下段支撑臂',lower_arm(),2,'flat_y'),
 ('03_upper_arm','上段轴承臂',upper_arm(),2,'flat_y'),
 ('04_splice_plate','接缝夹板',splice(),4,'flat_y'),
 ('05_bearing_cover','轴承端盖',cover(),2,'flat_y'),
 ('06_rod_hub','摆杆与轴夹持座',rod_hub(),1,'flat_y'),
 ('07_encoder_rail','编码器滑轨支架',encoder_rail(),1,'upright'),
 ('08_encoder_carrier','编码器模块背板',encoder_carrier(),1,'flat_y'),
 ('09_magnet_cup','轴端磁铁座',magnet_cup(),1,'flat_y'),
 ('10_fit_coupon','配合试打块',coupon(),1,'upright')
]
parts={s[0]:s[2] for s in specs}
colors={'print':'#198b9a','join':'#395269','hub':'#ed9c45','sensor':'#9665b3','metal':'#b9c5ce','rod':'#303e4c','pcb':'#40885c','magnet':'#ce5555','ref':'#d5dce0'}
scene=[]
def add(name,shape,color,group='fixed',reference=False):
    scene.append(dict(name=name,shape=shape,color=color,group=group,reference=reference))

for sign,side in [(-1,'L'),(1,'R')]:
    add('base_'+side,parts['01_base_shoe'].translate((0,sign*C,0)),colors['print'])
    add('lower_'+side,parts['02_lower_arm'].translate((0,sign*C,0)),colors['print'])
    u=parts['03_upper_arm']
    if sign<0: u=u.rotate((0,0,0),(0,0,1),180)
    add('upper_'+side,u.translate((0,sign*C,0)),colors['print'])
    for face in [-1,1]:
        add(f'splice_{side}_{face}',parts['04_splice_plate'].translate((0,sign*C+face*(T/2+3),0)),colors['join'])
    add('cover_'+side,parts['05_bearing_cover'].translate((0,sign*(C+T/2+1.5),0)),colors['join'])
    y0=C+T/2-7.1
    b=cy(22,y0,7).cut(cy(8,y0-1,9))
    if sign<0:b=b.rotate((0,0,H),(0,0,H+1),180)
    add('608_'+side,b,colors['metal'],'hardware')
    y0=C+T/2+0.5
    # Reference hardware updated to the user's selected 8 x 14 x 6 collar.
    col=cy(14,y0,6).cut(cy(8,y0-1,8))
    washer=cy(11,y0-0.5,0.5).cut(cy(8.2,y0-1,2))
    if sign<0:
        col=col.rotate((0,0,H),(0,0,H+1),180)
        washer=washer.rotate((0,0,H),(0,0,H+1),180)
    add('collar_'+side,col,colors['metal'],'rotor')
    add('shim_'+side,washer,colors['metal'],'rotor')
add('shaft_8x190',cy(7.98,C-155,190),colors['metal'],'rotor')
add('rod_hub',parts['06_rod_hub'],colors['hub'],'rotor')
tube_len=P['rod_pivot_to_tip']-P['tube_insertion_bottom_above_pivot']
tube=cyl(4,tube_len,0,0,H+20).cut(cyl(3,tube_len+2,0,0,H+19))
add('tube_8x6x260',tube,colors['rod'],'rotor')
add('encoder_rail',parts['07_encoder_rail'],colors['sensor'])
add('encoder_carrier',parts['08_encoder_carrier'],colors['sensor'])
add('magnet_cup',parts['09_magnet_cup'],colors['hub'],'rotor')
add('magnet_6x3',cy(6,C+42,3),colors['magnet'],'rotor')
# PCB placement uses package face at magnet-face + specified nominal air gap.
mag_face=C+45
pcb_front=mag_face+P['nominal_magnet_to_package_gap']+1.0
pcb=box(P['pcb_width'],1.6,P['pcb_height'],0,pcb_front+0.8,H-P['pcb_height']/2)
for x in [-P['pcb_hole_pitch_x']/2,P['pcb_hole_pitch_x']/2]:
    for z in [H-P['pcb_hole_pitch_z']/2,H+P['pcb_hole_pitch_z']/2]:pcb=pcb.cut(cy(2.8,pcb_front-1,4,x,z))
add('PCB_REFERENCE_NOT_VENDOR_SPEC',pcb,colors['pcb'],'reference',True)
add('AS5048_PACKAGE_REFERENCE',box(5,1,6,0,pcb_front-0.5,H-3),colors['rod'],'reference',True)
for x in [-38,38]:add('M4_tie_rod_'+str(x),cy(4,-90,180,x,20),colors['metal'],'hardware')
deck=box(280,240,3,z=-3)
add('CHASSIS_DECK_ENVELOPE_ONLY',deck,colors['ref'],'reference',True)

def mesh_data(shape,tolerance=0.18):
    vertices,faces=shape.tessellate(tolerance,0.15)
    return np.array([v.toTuple() for v in vertices]),np.array(faces)

def to_print(s,orientation):
    if orientation=='flat_y':s=s.rotate((0,0,0),(1,0,0),90)
    b=s.BoundingBox()
    return s.translate((-b.xmin,-b.ymin,-b.zmin))

def write_3mf(mesh,path,name):
    # Vendor-neutral geometry-only 3MF; no unverified slicer profiles/G-code.
    from xml.etree.ElementTree import Element,SubElement,tostring
    ns='http://schemas.microsoft.com/3dmanufacturing/core/2015/02'
    model=Element('model',{'unit':'millimeter','xml:lang':'en-US','xmlns':ns})
    md=SubElement(model,'metadata',{'name':'Title'});md.text=name
    resources=SubElement(model,'resources');obj=SubElement(resources,'object',{'id':'1','type':'model'})
    xm=SubElement(obj,'mesh');vs=SubElement(xm,'vertices');ts=SubElement(xm,'triangles')
    for v in mesh.vertices:SubElement(vs,'vertex',dict(zip(['x','y','z'],[f'{a:.5f}' for a in v])))
    for f in mesh.faces:SubElement(ts,'triangle',dict(zip(['v1','v2','v3'],[str(a) for a in f])))
    build=SubElement(model,'build');SubElement(build,'item',{'objectid':'1'})
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml','<?xml version="1.0"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr('_rels/.rels','<?xml version="1.0"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr('3D/3dmodel.model',tostring(model,encoding='utf-8',xml_declaration=True))

report={'revision':P['revision'],'units':'mm','parts':[],'checks':{},'limitations':[
 'Prototype geometry only: no physical print, load, vibration or control validation.',
 'Encoder PCB is a configurable reference, not a verified commercial module.',
 'Assembly omits most fastener detail. No G-code or Bambu slicer settings are supplied.'
]}
for ident,label,shape,qty,orient in specs:
    assert shape.isValid(), f'Invalid BRep {ident}'
    assert len(shape.Solids())==1, f'Disconnected {ident}'
    ps=to_print(shape,orient)
    cq.exporters.export(ps,str(OUT/'stl'/f'{ident}.stl'),tolerance=0.05,angularTolerance=0.1)
    cq.exporters.export(shape,str(OUT/'step'/f'{ident}.step'))
    mesh=trimesh.load_mesh(OUT/'stl'/f'{ident}.stl',process=True)
    assert mesh.is_watertight and mesh.is_volume, f'Not printable closed volume {ident}'
    assert max(mesh.extents[:2])<=170.1 and mesh.extents[2]<180, f'Print exceeds envelope {ident}'
    write_3mf(mesh,OUT/'3mf'/f'{ident}.3mf',ident)
    report['parts'].append({'id':ident,'name':label,'quantity':qty,'size_print_xyz_mm':np.round(mesh.extents,2).tolist(),'solid_volume_cm3':round(shape.Volume()/1000,2),'brep_valid':True,'watertight':True,'positive_volume':True,'orientation':orient})
    print('EXPORTED',ident,np.round(mesh.extents,2),flush=True)

assembly=cq.Assembly(name='Pendulum_A1mini_V2_rounded')
view=[]
for o in scene:
    rgb=tuple(int(o['color'][i:i+2],16)/255 for i in (1,3,5))
    assembly.add(o['shape'],name=o['name'],color=cq.Color(*rgb))
    vertices,faces=mesh_data(o['shape'])
    view.append({k:o[k] for k in ['name','color','group','reference']} | {'v':np.round(vertices,3).tolist(),'f':faces.tolist()})
assembly.save(str(OUT/'assembly.step'))
(OUT/'preview'/'scene.json').write_text(json.dumps(view,separators=(',',':')),encoding='utf-8')

# Check static printable pairs for unintended overlap; intended butt faces have 0 volume.
static=[o for o in scene if o['group']=='fixed']
overlaps=[]
for i,a in enumerate(static):
    for b in static[i+1:]:
        bb1,bb2=a['shape'].BoundingBox(),b['shape'].BoundingBox()
        if any(getattr(bb1,k+'max') <= getattr(bb2,k+'min')+1e-5 or getattr(bb2,k+'max') <= getattr(bb1,k+'min')+1e-5 for k in 'xyz'):continue
        vol=a['shape'].intersect(b['shape']).Volume()
        if vol>0.01:overlaps.append([a['name'],b['name'],round(vol,4)])
report['checks']['static_printed_part_overlaps_mm3']=overlaps
assert not overlaps, f'Static collisions {overlaps}'

# Full 360-degree central rotor sweep: broad phase exact Y-separation from towers,
# then sample the central moving hub/tube against any remaining overlapping static part.
rotor=parts['06_rod_hub'].fuse(tube)
moving_y=rotor.BoundingBox()
candidates=[o for o in static if o['shape'].BoundingBox().ymax>moving_y.ymin and o['shape'].BoundingBox().ymin<moving_y.ymax]
maxvol=0
for angle in range(0,360,5):
    r=rotor.rotate((0,0,H),(0,1,H),angle)
    for o in candidates:maxvol=max(maxvol,r.intersect(o['shape']).Volume())
assert maxvol<0.01
# A cylinder enclosing the eccentric magnet cup covers every rotation continuously.
cup=parts['09_magnet_cup'];cb=cup.BoundingBox()
cup_r=math.hypot(max(abs(cb.xmin),abs(cb.xmax)),max(abs(cb.zmin-H),abs(cb.zmax-H)))
cup_envelope=cy(cup_r*2,cb.ymin,cb.ylen)
cup_obstacles=static+[o for o in scene if o['name'] in ['PCB_REFERENCE_NOT_VENDOR_SPEC','AS5048_PACKAGE_REFERENCE']]
cup_hits=[]
for o in cup_obstacles:
    vol=cup_envelope.intersect(o['shape']).Volume()
    if vol>.01:cup_hits.append([o['name'],round(vol,4)])
assert not cup_hits,f'Magnet cup sweep collision {cup_hits}'
# Reference bottom tie rods also stay below the pendulum swept volume.
tie_z_top=22
# Conservative swept radius includes tube radius and holder corners.
tip=P['rod_pivot_to_tip']
swept_radius=math.sqrt(tip**2+4**2)
report['checks'].update({
 'all_printed_parts_fit_180mm_bed_with_5mm_border':True,
 'central_rotor_sweep_step_degrees':5,
 'central_rotor_static_overlap_max_mm3':round(maxvol,6),
 'magnet_cup_full_rotation_envelope_collisions_mm3':cup_hits,
 'magnet_cup_swept_envelope_radius_mm':round(cup_r,3),
 'conservative_pendulum_to_tie_rod_vertical_clearance_mm':round(H-swept_radius-tie_z_top,3),
 'central_rotor_candidates_after_exact_y_separation':len(candidates),
 'conservative_rotor_to_deck_clearance_mm':round(H-swept_radius,3),
 'central_rotor_to_tower_inner_face_y_clearance_mm':round(C-T/2-16,3),
 'bearing_seat_diametral_clearance_mm':P['bearing_seat_diameter']-22,
 'magnet_to_package_nominal_gap_mm':P['nominal_magnet_to_package_gap'],
 'encoder_carrier_rail_hole_y_range_mm':[C+49-(30-3.4)/2,C+49+(30-3.4)/2],
 'printed_functional_piece_count':sum(s[3] for s in specs if s[0]!='10_fit_coupon')
})
(OUT/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
with (OUT/'print_manifest.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['文件','名称','数量','打印XYZ尺寸mm','实体体积cm3'])
    for p in report['parts']:w.writerow([p['id'],p['name'],p['quantity'],' x '.join(map(str,p['size_print_xyz_mm'])),p['solid_volume_cm3']])
print('VERIFIED',json.dumps(report['checks'],ensure_ascii=False),flush=True)

if __name__=='__main__':
    import make_preview
    make_preview.render(ROOT,OUT,view,P)
