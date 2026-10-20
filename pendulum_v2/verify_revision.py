"""Check V2 interfaces against immutable V1 solids, not just new mesh syntax."""
from pathlib import Path
import json, hashlib
import cadquery as cq
root=Path(__file__).resolve().parent
old=root.parent/'pendulum_v1'/'output'
new=root/'output'
def read(folder,ident):return cq.importers.importStep(str(folder/'step'/f'{ident}.step')).val()
def symdiff(a,b):return a.cut(b).Volume()+b.cut(a).Volume()
results={}
for ident in ['01_base_shoe','10_fit_coupon']:
    delta=symdiff(read(old,ident),read(new,ident))
    assert delta<1e-6,(ident,delta)
    results[ident+'_geometry_difference_mm3']=delta
    results[ident+'_stl_byte_identical']=hashlib.sha256((old/'stl'/f'{ident}.stl').read_bytes()).digest()==hashlib.sha256((new/'stl'/f'{ident}.stl').read_bytes()).digest()
clip=cq.Solid.makeBox(140,20,26,cq.Vector(-70,-10,8))
delta=symdiff(read(old,'02_lower_arm').intersect(clip),read(new,'02_lower_arm').intersect(clip))
assert delta<1e-6
results['lower_arm_z8_to_z34_base_interface_difference_mm3']=delta
lower_added=read(new,'02_lower_arm').cut(read(old,'02_lower_arm')).Volume()
lower_removed=read(old,'02_lower_arm').cut(read(new,'02_lower_arm')).Volume()
assert lower_added>0 and lower_removed<1e-6
results['lower_window_fillet_added_material_mm3']=lower_added
results['lower_arm_removed_material_mm3']=lower_removed
# Original mating faces, bolt centers, shaft and bearing-seat clearances.
upper=read(new,'03_upper_arm')
checks=[]
def cylinder(r,p,d,h):return cq.Solid.makeCylinder(r,h,cq.Vector(*p),cq.Vector(*d))
checks.append(('bearing_seat',upper,cylinder(11.1,(0,-1.2,320),(0,1,0),7.2)))
checks.append(('bearing_shoulder_clearance',upper,cylinder(8.3,(0,-6,320),(0,1,0),12)))
for x in [-38,38]:checks.append((f'base_tie_hole_{x}',read(new,'02_lower_arm'),cylinder(2.25,(x,-6,20),(0,1,0),12)))
for x in [-16,16]:
    checks.append((f'lower_splice_{x}',read(new,'02_lower_arm'),cylinder(2.25,(x,-6,155),(0,1,0),12)))
    checks.append((f'upper_splice_{x}',upper,cylinder(2.25,(x,-6,195),(0,1,0),12)))
checks.append(('hub_shaft',read(new,'06_rod_hub'),cylinder(4.1,(0,-16,320),(0,1,0),32)))
checks.append(('hub_tube',read(new,'06_rod_hub'),cylinder(4.15,(0,0,340),(0,0,1),32)))
for x in [-23,23]:
    checks.append((f'carrier_screw_head_access_{x}',read(new,'08_encoder_carrier'),cylinder(3.1,(x,109,304.01),(0,0,1),60)))
for name,part,empty in checks:
    vol=part.intersect(empty).Volume()
    assert vol<1e-6,(name,vol)
results['interface_and_fastener_access_checks']=[n for n,_,_ in checks]
results['fillets_mm']={'lower_window':4,'upper_window':3,'hub_T_root':2,'rail_root':2.5,'carrier_root':2,'splice_outline':4,'cover_outline':3}
results['not_claimed']='No FEA, fatigue life, physical fit, or measured strength improvement.'
(new/'revision_compatibility.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(results,ensure_ascii=False,indent=2))
