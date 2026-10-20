from pathlib import Path
import ast,json,math,csv,zipfile,base64
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
features=[h for h in features if h['group'] not in ['MOTOR_M4','MOTOR520_M3','SIDE_M3']]
for h in features:
    if h['group']=='SENSOR_M3':h['x']-=math.copysign(10,h['x'])
for x in [-140,140]:
    for y in [-85,-60,60,85]:features.append({'group':'SIDE_M3','kind':'circle','x':x,'y':y,'diameter':3.4})
for sx in [-1,1]:
    for sy in [-1,1]:
        for dx in [-8,8]:
            for dy in [-7,7]:features.append({'group':'WHEELSET_DIRECT_M3','kind':'circle','x':sx*118+dx,'y':sy*84+dy,'diameter':3.4})
for i,h in enumerate(features,1):h['id']=f'H{i:03d}'
assert len(features)==84
plate=rounded_plate(300,200,4,-4,10)
outer_wire=cq.Workplane(obj=plate).faces('>Z').val().outerWire()
cuts=[cut_for(h) for h in features]
edge_clearance=min(outer_wire.distance(c) for c in cuts)
assert edge_clearance>5.0,edge_clearance
for c in cuts:plate=plate.cut(c)
plate=plate.clean();assert plate.isValid() and len(plate.Solids())==1
for i,a in enumerate(cuts):
    for b in cuts[i+1:]:assert a.intersect(b).Volume()<1e-6
name='chassis_V3_direct_14x16_300x200x4'
cq.exporters.export(plate,str(OUT/'step'/f'{name}.step'))
cq.exporters.export(plate.translate((150,100,4)),str(OUT/'stl_reference'/f'{name}.stl'),tolerance=.06,angularTolerance=.12)
m=trimesh.load_mesh(OUT/'stl_reference'/f'{name}.stl',process=True);assert m.is_watertight and m.is_volume
dxf_file(name,300,200,10,features)
assert cq.importers.importStep(str(OUT/'step'/f'{name}.step')).val().isValid()
base=cq.importers.importStep(str(R.parent/'pendulum_v3/output/step/01_base_shoe.step')).val()
for sy in [-1,1]:
    placed=base.translate((0,sy*65,0));assert placed.intersect(plate).Volume()<1e-6
    for x in [-55,55]:
        for yy in [-23,23]:assert placed.intersect(cylinder(5,12,x,sy*65+yy,-1)).Volume()<1e-6
for cx in [-118,118]:
    for cy in [-84,84]:
        for dx in [-8,8]:
            for dy in [-7,7]:assert plate.intersect(cylinder(3,6,cx+dx,cy+dy,-5)).Volume()<1e-6
with (OUT/'hole_coordinates_mm.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['编号','用途','形状','X','Y','直径','总长','槽宽','角度'])
    for h in features:w.writerow([h['id'],h['group'],h['kind'],h['x'],h['y'],h.get('diameter',''),h.get('length',''),h.get('width',''),h.get('angle','')])
(OUT/'hole_features.json').write_text(json.dumps(features,ensure_ascii=False,indent=2),encoding='utf-8')
# Actual-scale SVG drawing in mm, with dimension/purpose labels in the margins.
svg=['<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="1060" viewBox="0 0 1400 1060"><rect width="1400" height="1060" fill="white"/><g font-family="Microsoft YaHei,sans-serif" fill="#213547">',
 '<text x="60" y="60" font-size="30" font-weight="600">底板 V3 · 新轮组直接安装</text>',
 '<text x="60" y="104" font-size="21">300×200×4 mm · 四组 16(X)×14(Y) 孔距 · 每组四个 Ø3.4 孔</text>']
sc=3.4;ox=680;oy=510
svg.append(f'<rect x="{ox-150*sc}" y="{oy-100*sc}" width="{300*sc}" height="{200*sc}" rx="{10*sc}" fill="#f4f6ef" stroke="#213547" stroke-width="2"/>')
for sy in [-1,1]:svg.append(f'<rect x="{ox-80*sc}" y="{oy-(sy*65+30)*sc}" width="{160*sc}" height="{60*sc}" fill="#f3d3ac" fill-opacity="0.25" stroke="#b07e46" stroke-dasharray="7 5"/>')
for h in features:
    x,y=ox+h['x']*sc,oy-h['y']*sc;c='#c14141' if h['group']=='WHEELSET_DIRECT_M3' else '#246e87'
    if h['kind']=='circle':svg.append(f'<circle cx="{x}" cy="{y}" r="{h["diameter"]/2*sc}" fill="white" stroke="{c}" stroke-width="2"/>')
    else:
        l,w=h['length']*sc,h['width']*sc
        svg.append(f'<rect x="{-l/2}" y="{-w/2}" width="{l}" height="{w}" rx="{w/2}" transform="translate({x},{y}) rotate({-h["angle"]})" fill="white" stroke="{c}" stroke-width="1.5"/>')
for x in [-118,118]:
    for y in [-84,84]:
        xx,yy=ox+x*sc,oy-y*sc
        svg.append(f'<rect x="{xx-14*sc}" y="{yy-13*sc}" width="{28*sc}" height="{26*sc}" fill="none" stroke="#c14141" stroke-width="1.5" stroke-dasharray="5 4"/>')
svg.extend(['<text x="60" y="925" font-size="22" fill="#c14141">红孔：整套新轮组直接固定；不需要独立小转接片。</text>',
 '<text x="60" y="969" font-size="20">保留原倒立摆、板卡、IMU、电池和传感器接口；原 520 / 转接片孔已移除。</text>',
 '<text x="60" y="1014" font-size="19">孔位已检查；轴心高度和实际轮胎间隙仍需实物确认。加工请使用 DXF 毫米尺寸。</text>','</g></svg>'])
s='\n'.join(svg);(OUT/'preview/chassis_V3_top.svg').write_text(s,encoding='utf-8')
(OUT/'preview/chassis_V3_top.png').write_bytes(resvg_py.svg_to_bytes(svg_string=s,font_files=['C:/Windows/Fonts/msyh.ttc']))
report={'revision':'V3-300x200','size_mm':[300,200,4],'minimum_cutout_to_plate_edge_mm':round(edge_clearance,3),'direct_mount':True,'feature_count':84,'motor_holes':16,'motor_hole_pitch_x_y_mm':[16,14],'motor_mount_centers_xy_mm':[[x,y] for x in [-118,118] for y in [-84,84]],'motor_hole_diameter_mm':3.4,'solid_valid':True,'stl_closed':True,'dxf_readback':True,'old_pendulum_base_compatible':True,'hole_cutters_overlap':False,'axle_height_confirmed':False,'physical_fit_tested':False,'FR4_estimated_mass_g_at_1_9':round(plate.Volume()*.0019,1)}
(OUT/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

# Generate a portable, dependency-free browser preview.  The mesh and the local
# Three.js runtime are embedded so the file still works after it is copied out
# of this repository and opened directly from disk.
three_js=(old/'output/preview/three.min.js').read_text(encoding='utf-8')
stl_b64=base64.b64encode((OUT/'stl_reference'/f'{name}.stl').read_bytes()).decode('ascii')
svg_b64=base64.b64encode((OUT/'preview/chassis_V3_top.svg').read_bytes()).decode('ascii')
report_json=json.dumps(report,ensure_ascii=False)
html_template=r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>底板 V3 300×200×4 mm · 离线三维预览</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#eef2f1;color:#18333d;font:15px system-ui,"Microsoft YaHei",sans-serif}header{padding:18px 24px;background:#fff;border-bottom:1px solid #cedbd8}h1{font-size:22px;margin:0 0 7px}header p{margin:0;color:#5c737a;line-height:1.55}main{max-width:1500px;margin:auto;display:grid;grid-template-columns:minmax(0,1fr) 310px;min-height:calc(100vh - 100px)}.view{position:relative;min-height:650px;background:linear-gradient(#f9fbfa,#dce7e5)}canvas{display:block;width:100%;height:100%;position:absolute;inset:0;touch-action:none}.toolbar{position:absolute;z-index:2;top:14px;left:14px;display:flex;gap:7px;flex-wrap:wrap}button{border:1px solid #afc5c1;border-radius:5px;background:#fff;color:#18333d;padding:7px 11px;cursor:pointer}button:hover,button[aria-pressed=true]{background:#18333d;color:#fff}.hint{position:absolute;z-index:2;left:15px;bottom:12px;padding:5px 8px;background:#ffffffcc;color:#5c737a;font-size:12px;border-radius:4px}aside{padding:24px;background:#fff;border-left:1px solid #cedbd8}h2{font-size:16px;margin:0 0 13px}.spec{display:grid;grid-template-columns:1fr auto;gap:9px 12px;line-height:1.55}.spec dt{color:#607980}.spec dd{margin:0;text-align:right;font-variant-numeric:tabular-nums}.rule{border:0;border-top:1px solid #d9e3e0;margin:20px 0}.key{display:flex;gap:9px;align-items:center;margin:10px 0;color:#526c73}.swatch{width:14px;height:14px;border-radius:3px;background:#8eafa0}.swatch.red{background:#c14141}.note{color:#607980;line-height:1.7;font-size:13px}.topview{display:block;width:100%;margin-top:16px;border:1px solid #d9e3e0}.topview[hidden]{display:none}@media(max-width:780px){main{display:block}.view{min-height:540px}aside{border-left:0;border-top:1px solid #cedbd8;padding:18px}header{padding:15px 17px}h1{font-size:19px}}
</style></head><body>
<header><h1>底板 V3 · 新轮组直接安装</h1><p>300×200×4 mm · 84 个孔/槽 · 14×16 轮组孔距 · 离线独立预览</p></header>
<main><section class="view" aria-label="可旋转的三维底板预览"><canvas id="view"></canvas><div class="toolbar"><button id="iso" aria-pressed="true">等轴视</button><button id="top">顶视</button><button id="reset">重置</button><button id="edges" aria-pressed="true">边线</button></div><div class="hint">拖动旋转 · 滚轮缩放</div></section>
<aside><h2>本版参数</h2><dl class="spec"><dt>外形</dt><dd>300×200×4 mm</dd><dt>外角</dt><dd>R10</dd><dt>特征总数</dt><dd>84</dd><dt>轮组孔组</dt><dd>4 × 4</dd><dt>孔径</dt><dd>Ø3.4 mm</dd><dt>孔组中心</dt><dd>X±118 / Y±84</dd><dt>最小边缘余量</dt><dd>7.3 mm</dd><dt>FR4估算质量</dt><dd>444.1 g</dd></dl><hr class="rule"><div class="key"><i class="swatch"></i>底板实体</div><div class="key"><i class="swatch red"></i>新轮组直装孔区（顶视图）</div><img class="topview" alt="底板 V3 孔位顶视图" src="data:image/svg+xml;base64,__SVG__"><hr class="rule"><p class="note">STEP、DXF、封闭 STL 与孔位表在同名 ZIP 内。轮组安装面到轴心高度、轮胎和支架实际间隙仍未确认；本预览不代表已完成实物试装。</p></aside></main>
<script>__THREE__</script>
<script>
const STL_B64='__STL__';
const canvas=document.getElementById('view');
const status={size:[300,200,4],features:84};
function decodeSTL(data){const raw=atob(data),bytes=new Uint8Array(raw.length);for(let i=0;i<raw.length;i++)bytes[i]=raw.charCodeAt(i);const dv=new DataView(bytes.buffer),n=dv.getUint32(80,true),v=new Float32Array(n*9);for(let i=0;i<n;i++){const p=84+i*50;for(let j=0;j<9;j++)v[i*9+j]=dv.getFloat32(p+12+j*4,true)}const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(v,3));g.computeVertexNormals();return g}
const renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));renderer.setClearColor(0x000000,0);const scene=new THREE.Scene();scene.add(new THREE.HemisphereLight(0xffffff,0x516b69,2.1));const sun=new THREE.DirectionalLight(0xffffff,2.4);sun.position.set(-240,-260,500);scene.add(sun);const target=new THREE.Vector3(150,100,2);const geometry=decodeSTL(STL_B64);const material=new THREE.MeshStandardMaterial({color:0x8eafa0,roughness:.78,metalness:.08,side:THREE.DoubleSide});const plate=new THREE.Mesh(geometry,material);scene.add(plate);const edgeMaterial=new THREE.LineBasicMaterial({color:0x36534e,transparent:true,opacity:.52});const edgeLines=new THREE.LineSegments(new THREE.EdgesGeometry(geometry,24),edgeMaterial);scene.add(edgeLines);const camera=new THREE.PerspectiveCamera(34,1,.1,3000);let yaw=-.88,pitch=.62,radius=510,drag=null,showEdges=true;
function cameraUpdate(){const cp=Math.cos(pitch);camera.position.set(target.x+radius*cp*Math.cos(yaw),target.y+radius*cp*Math.sin(yaw),target.z+radius*Math.sin(pitch));camera.lookAt(target)}function draw(){const r=canvas.getBoundingClientRect(),w=Math.max(1,r.width),h=Math.max(1,r.height);renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();cameraUpdate();edgeLines.visible=showEdges;renderer.render(scene,camera);canvas.dataset.rendered='true';window.viewerRenderSucceeded&&window.viewerRenderSucceeded()}function preset(top){if(top){yaw=0;pitch=1.56;radius=520}else{yaw=-.88;pitch=.62;radius=510}draw()}document.getElementById('iso').onclick=()=>{document.getElementById('iso').setAttribute('aria-pressed','true');preset(false)};document.getElementById('top').onclick=()=>{document.getElementById('iso').setAttribute('aria-pressed','false');preset(true)};document.getElementById('reset').onclick=()=>{document.getElementById('iso').setAttribute('aria-pressed','true');preset(false)};document.getElementById('edges').onclick=()=>{showEdges=!showEdges;document.getElementById('edges').setAttribute('aria-pressed',String(showEdges));draw()};canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};canvas.onpointermove=e=>{if(drag){yaw-=(e.clientX-drag[0])*.009;pitch=Math.max(-1.42,Math.min(1.55,pitch+(e.clientY-drag[1])*.007));drag=[e.clientX,e.clientY];draw()}};canvas.onpointerup=()=>drag=null;canvas.onpointercancel=()=>drag=null;canvas.addEventListener('wheel',e=>{e.preventDefault();radius=Math.max(260,Math.min(1200,radius*Math.exp(e.deltaY*.001)));draw()},{passive:false});new ResizeObserver(draw).observe(canvas.parentElement);draw();
</script></body></html>'''
html=(html_template.replace('__THREE__',three_js).replace('__STL__',stl_b64).replace('__SVG__',svg_b64).replace('__REPORT__',report_json))
preview_root=R.parent/'pendulum_底板V3_300x200_独立预览.html'
preview_root.write_text(html,encoding='utf-8')
print(json.dumps(report))
