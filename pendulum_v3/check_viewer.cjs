// Node-only checks. This does not launch a browser or test the host GPU.
const fs=require('fs'),path=require('path'),vm=require('vm'),assert=require('assert');
const out=path.join(__dirname,'output','preview');
const html=fs.readFileSync(path.join(out,'assembly_viewer.html'),'utf8');
assert(!/<script\s[^>]*src=/i.test(html),'Viewer must not need sibling scripts');
const image=html.match(/id="fallbackImage"[^>]*src="data:image\/png;base64,([A-Za-z0-9+/=]+)"/);
assert(image,'Static fallback must exist in original HTML');
assert(Buffer.from(image[1],'base64').equals(fs.readFileSync(path.join(out,'assembly.png'))));
assert(!/id="fallbackImage"[^>]*\bhidden\b/.test(html),'Fallback visible before JavaScript');
const scripts=[...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m=>m[1]);
assert.equal(scripts.length,3);
scripts.forEach((s,i)=>new vm.Script(s,{filename:'inline-'+i+'.js'}));
function environment(){
  const elements={},frames=[],events={};
  for(const id of ['view','fallbackImage','loadStatus','all','detail','front','side','reset','angle','angleText','refs','sweep']){
    elements[id]={style:{visibility:id==='view'?'hidden':''},dataset:{},hidden:false,disabled:true,textContent:'',checked:id==='refs',value:'0',setAttribute(k,v){this[k]=v},addEventListener(){},getContext(){return null},setPointerCapture(){},parentElement:{getBoundingClientRect(){return {width:900,height:640}}}};
  }
  const controls=['all','detail','front','side','reset','angle','refs','sweep'].map(id=>elements[id]);
  const doc={getElementById:id=>elements[id],querySelectorAll:()=>controls,documentElement:{dataset:{}}};
  const ctx={document:doc,devicePixelRatio:1,console:{log(){},warn(){},error(){}},requestAnimationFrame:f=>frames.push(f),ResizeObserver:class{constructor(f){this.f=f}observe(){this.f()}},addEventListener:(k,f)=>events[k]=f};
  ctx.window=ctx;ctx.self=ctx;vm.createContext(ctx);
  function flush(){let count=0;while(frames.length){assert(++count<10);frames.shift()()}}
  return {ctx,e:elements,frames,doc,controls,flush};
}
// Exercise the real Three.js initialization with no graphics context.
const noGL=environment();scripts.forEach(s=>vm.runInContext(s,noGL.ctx));
assert.equal(noGL.doc.documentElement.dataset.viewerMode,'static');
assert.equal(noGL.e.fallbackImage.hidden,false);
assert(noGL.e.loadStatus.textContent.includes('已显示装配图'));
assert(noGL.controls.every(c=>c.disabled));
// Missing library must also produce a readable fallback, not a blank viewport.
const missing=environment();vm.runInContext(scripts[0],missing.ctx);vm.runInContext(scripts[2],missing.ctx);
assert.equal(missing.doc.documentElement.dataset.viewerMode,'static');
// Exercise interaction logic with real scene/geometry classes and a renderer stub.
const normal=environment();vm.runInContext(scripts[0],normal.ctx);vm.runInContext(scripts[1],normal.ctx);
let renders=0;
normal.ctx.THREE.WebGLRenderer=class{setPixelRatio(){}setSize(){}render(){renders++}};
vm.runInContext(scripts[2],normal.ctx);normal.flush();
assert.equal(normal.doc.documentElement.dataset.viewerMode,'3d');
assert.equal(normal.e.fallbackImage.hidden,true);
assert(normal.controls.every(c=>!c.disabled));
normal.e.angle.value='180';normal.e.angle.oninput({target:normal.e.angle});normal.flush();
assert.equal(normal.e.view.dataset.angle,'180');
normal.e.detail.onclick();normal.flush();assert.equal(normal.e.detail['aria-pressed'],'true');
normal.e.reset.onclick();normal.flush();assert.equal(normal.e.view.dataset.angle,'0');assert(renders>=4);
normal.ctx.showViewerFallback('模拟运行时渲染失败');
assert.equal(normal.e.fallbackImage.hidden,false);
const result={single_file_no_external_scripts:true,embedded_png_verified:true,inline_scripts_parse:true,no_graphics_context_fallback:true,missing_library_fallback:true,interaction_logic_with_renderer_stub:true,actual_browser_or_GPU_tested:false,reason:'Browser tool blocked the local file URL; no alternate browser transport used.'};
fs.writeFileSync(path.join(__dirname,'output','viewer_verification.json'),JSON.stringify(result,null,2));
console.log(JSON.stringify(result));
