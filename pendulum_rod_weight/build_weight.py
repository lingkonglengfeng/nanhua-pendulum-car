"""Split-collar adjustable pendulum weight accessory. All dimensions mm."""
from pathlib import Path
import json,math,csv,zipfile,hashlib,sys
import cadquery as cq
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parent;OUT=ROOT/'output'
P=json.loads((ROOT/'parameters.json').read_text(encoding='utf-8'))
for name in ['stl','step','3mf','preview']:(OUT/name).mkdir(parents=True,exist_ok=True)
V=cq.Vector;H=P['pivot_height'];CH=P['clamp_height'];D=P['clamp_depth_y'];W=P['clamp_width_x']
N=P['default_washer_count'];WT=P['washer_thickness'];POS=P['lower_collar_bottom_from_pivot']
def box(x,y,z,centerx=0,centery=0,z0=0):return cq.Solid.makeBox(x,y,z,V(centerx-x/2,centery-y/2,z0))
def cyl(d,h,x=0,y=0,z=0,axis=(0,0,1)):return cq.Solid.makeCylinder(d/2,h,V(x,y,z),V(*axis))
def cy(d,y0,h,x=0,z=0):return cyl(d,h,x,y0,z,(0,1,0))
def hex_y(x,z,y0,depth,flats):
    r=flats/math.sqrt(3);pts=[(x+r*math.cos(i*math.pi/3),z+r*math.sin(i*math.pi/3)) for i in range(6)]
    pl=cq.Plane(origin=(0,y0+depth,0),xDir=(1,0,0),normal=(0,-1,0))
    return cq.Workplane(pl).polyline(pts).close().extrude(depth).val()
body=box(W,D,CH)
body=cq.Workplane(obj=body).edges('|Z').fillet(P['outer_corner_radius']).val()
body=body.cut(cyl(P['clamp_bore_diameter'],CH+2,z=-1))
body=cq.Workplane(obj=body).edges('%Circle').chamfer(.2).val()
for x in [-P['bolt_center_x'],P['bolt_center_x']]:body=body.cut(cy(P['bolt_clearance_diameter'],-D,D*2,x,CH/2))
head=body.intersect(box(W+2,D,CH+2,centery=P['split_gap']/2+D/2,z0=-1)).clean()
nut=body.intersect(box(W+2,D,CH+2,centery=-P['split_gap']/2-D/2,z0=-1))
for x in [-P['bolt_center_x'],P['bolt_center_x']]:
    nut=nut.cut(hex_y(x,CH/2,-D/2-.1,P['nut_pocket_depth']+.1,P['nut_pocket_across_flats']))
nut=nut.clean()
specs=[('11_weight_clamp_head','配重夹环螺丝侧',head,2),('12_weight_clamp_nut','配重夹环螺母侧',nut,2)]

def stl_shape(s):
    b=s.BoundingBox();return s.translate((-b.xmin,-b.ymin,-b.zmin))
def write_3mf(mesh,path,name):
    from xml.etree.ElementTree import Element,SubElement,tostring
    root=Element('model',{'xmlns':'http://schemas.microsoft.com/3dmanufacturing/core/2015/02','unit':'millimeter'})
    resources=SubElement(root,'resources');o=SubElement(resources,'object',{'id':'1','type':'model','name':name});m=SubElement(o,'mesh');vs=SubElement(m,'vertices');ts=SubElement(m,'triangles')
    for v in mesh.vertices:SubElement(vs,'vertex',{k:f'{a:.6f}' for k,a in zip('xyz',v)})
    for f in mesh.faces:SubElement(ts,'triangle',{k:str(int(a)) for k,a in zip(['v1','v2','v3'],f)})
    SubElement(SubElement(root,'build'),'item',{'objectid':'1'})
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml','<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr('_rels/.rels','<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="r1" Target="/3D/3dmodel.model" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr('3D/3dmodel.model',tostring(root,encoding='utf-8',xml_declaration=True))
report={'units':'mm','parts':[],'checks':{}}
for ident,label,s,qty in specs:
    assert s.isValid() and len(s.Solids())==1
    cq.exporters.export(stl_shape(s),str(OUT/'stl'/f'{ident}.stl'),tolerance=.03,angularTolerance=.1)
    cq.exporters.export(s,str(OUT/'step'/f'{ident}.step'))
    mesh=trimesh.load_mesh(OUT/'stl'/f'{ident}.stl',process=True)
    assert mesh.is_volume and mesh.is_watertight and max(mesh.extents)<180
    write_3mf(mesh,OUT/'3mf'/f'{ident}.3mf',ident)
    report['parts'].append({'id':ident,'name':label,'qty':qty,'size_xyz':np.round(mesh.extents,3).tolist(),'solid_volume_cm3':round(s.Volume()/1000,4),'valid':True,'watertight':True})

def assembly_items(position,count):
    result=[]
    def add(name,s,color,material):result.append({'name':'weight_'+name,'shape':s,'color':color,'material':material,'group':'rotor','reference':False})
    def place(s,z,flip):
        if flip:s=s.rotate((0,0,0),(0,0,1),180)
        return s.translate((0,0,z))
    for side,z,flip in [('lower',H+position,False),('upper',H+position+CH+count*WT,True)]:
        add(side+'_head',place(head,z,flip),'#cd783c','printed')
        add(side+'_nut',place(nut,z,flip),'#cd783c','printed')
        for idx,x in enumerate([-P['bolt_center_x'],P['bolt_center_x']]):
            # M2.5x16: 0.5mm front washer, head 2.5 high, nut in the outer pocket.
            screw=cy(2.5,D/2+.5-16,16,x,CH/2).fuse(cy(4.5,D/2+.5,2.5,x,CH/2))
            washer=cy(5,D/2,.5,x,CH/2).cut(cy(2.7,D/2-.1,.7,x,CH/2))
            hn=hex_y(x,CH/2,-D/2+.1,2,5).cut(cy(2.6,-D,2*D,x,CH/2))
            add(side+f'_bolt{idx}',place(screw,z,flip),'#b9c5ce','metal')
            add(side+f'_smallwasher{idx}',place(washer,z,flip),'#b9c5ce','metal')
            add(side+f'_hexnut{idx}',place(hn,z,flip),'#aab6c0','metal')
    for n in range(count):
        s=cyl(P['washer_outer_diameter'],WT,z=H+position+CH+n*WT).cut(cyl(P['washer_inner_diameter'],WT+2,z=H+position+CH+n*WT-1))
        add('mass_washer_'+str(n+1),s,'#c7d0d5','steel_payload')
    return result

items=assembly_items(POS,N)
asm=cq.Assembly(name='Adjustable_Rod_Weight_8mm')
for o in items:
    assert o['shape'].isValid() and len(o['shape'].Solids())==1
    rgb=[int(o['color'][i:i+2],16)/255 for i in [1,3,5]]
    asm.add(o['shape'],name=o['name'],color=cq.Color(*rgb))
rod=cyl(8,260,z=H+20).cut(cyl(6,262,z=H+19))
asm.add(rod,name='existing_8mm_tube_reference',color=cq.Color(.18,.24,.29))
asm.save(str(OUT/'weight_assembly.step'))

overlaps=[]
for i,a in enumerate(items):
    assert a['shape'].intersect(rod).Volume()<1e-6
    for b in items[i+1:]:
        v=a['shape'].intersect(b['shape']).Volume()
        if v>.001:overlaps.append([a['name'],b['name'],v])
assert not overlaps,overlaps
# Compare accessory motion envelope with unchanged V3 structure. Rotating about Y
# preserves Y coordinates: all fixed print parts lie outside the center corridor.
v3=ROOT.parent/'pendulum_v3/output'
base_scene=json.loads((v3/'preview/scene.json').read_text(encoding='utf-8'))
static=[o for o in base_scene if o['group']=='fixed']
cases=[]
for pos in [P['position_min'],P['position_max']]:
    for count in [0,N,P['geometric_max_washer_count']]:
        trial=assembly_items(pos,count);radius=0;min_y=1e9;max_y=-1e9
        for a in trial:
            b=a['shape'].BoundingBox();min_y=min(min_y,b.ymin);max_y=max(max_y,b.ymax)
            radius=max(radius,math.hypot(max(abs(b.xmin),abs(b.xmax)),max(abs(b.zmin-H),abs(b.zmax-H))))
        assert radius<280 and H+pos>H+52+10
        for o in static:
            vv=np.array(o['v']);assert vv[:,1].max()<=min_y or vv[:,1].min()>=max_y,o['name']
        cases.append({'position':pos,'washers':count,'conservative_sweep_radius':round(radius,3),'bottom_clearance_to_deck':round(H-radius,3),'y_envelope':[min_y,max_y]})
        # Nut traps stay open to the outer face; every physical nut fits its pocket.
single_washer_mass=math.pi/4*(P['washer_outer_diameter']**2-P['washer_inner_diameter']**2)*WT*.00785
report['checks']={'default_part_overlaps_mm3':overlaps,'tube_nominal_radial_clearance_mm':(P['clamp_bore_diameter']-8)/2,'position_and_count_envelopes':cases,'nominal_full_accessory_height_mm':2*CH+N*WT,'washer_mass_theoretical_g':round(single_washer_mass,3),'printed_pair_solid_volume_cm3':round(2*(head.Volume()+nut.Volume())/1000,3),'v3_models_not_modified':True,'no_physical_grip_strength_or_dynamic_test':True}
readback=cq.importers.importStep(str(OUT/'weight_assembly.step'))
assert readback.val().isValid();report['checks']['step_readback_solids']=len(readback.solids().vals())
(OUT/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

def mesh(o):
    vs,fs=o['shape'].tessellate(.12,.12)
    return {k:o[k] for k in ['name','color','group','reference']}|{'v':[[round(a,4) for a in v.toTuple()] for v in vs],'f':[list(f) for f in fs]}
accessory=[mesh(o) for o in items]
fullscene=base_scene+accessory
(OUT/'preview/scene.json').write_text(json.dumps(fullscene,separators=(',',':')),encoding='utf-8')
(OUT/'preview/accessory_scene.json').write_text(json.dumps(accessory,separators=(',',':')),encoding='utf-8')
with (OUT/'print_manifest.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['零件','打印数量','外形mm']);[w.writerow([p['id'],p['qty'],p['size_xyz']]) for p in report['parts']]
print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
import preview_weight
preview_weight.render(ROOT,OUT,accessory,fullscene,P)
