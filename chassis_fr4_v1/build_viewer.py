"""Build a standalone, offline viewer with a visible static fallback."""
from pathlib import Path
import base64,json,re

def build(root,out,scene,p):
    template=(root/'viewer_template.html').read_text(encoding='utf-8').split('<script>')[0]
    template=template.replace('显示底盘与编码器占位件','显示底盘与编码器参考件').replace('编码器 PCB 为 30×30 mm、孔距 24×24 mm 的设计占位，选购后需核对。','AS5600 圆板 Ø32、板厚 1.6 mm；四孔中心按 17×17 mm 选在斜槽内，使用 6 mm 绝缘隔柱。图中未给出的插头外形仍需实物确认。')
    css='''<style>
    #view{visibility:hidden}.toolbar,.hint{z-index:2}
    #fallbackImage{position:absolute;inset:0;width:100%;height:100%;object-fit:contain}
    #fallbackImage[hidden]{display:none}
    #loadStatus{position:absolute;z-index:3;top:64px;left:16px;right:16px;padding:8px 10px;background:#fff;color:#385466;border:1px solid #cedbe3;border-radius:5px;font-size:13px;line-height:1.6}
    #loadStatus[hidden]{display:none}button:disabled,input:disabled{opacity:.5;cursor:default}
    </style>'''
    template=template.replace('</head>',css+'</head>')
    png=base64.b64encode((out/'preview/assembly.png').read_bytes()).decode('ascii')
    fallback='<img id="fallbackImage" alt="FR4 底盘与倒立摆装配图" src="data:image/png;base64,'+png+'"><div id="loadStatus" role="status">正在加载三维模型；若未加载，仍可查看装配图。建议用 Edge 或 Chrome 打开此 HTML 文件。</div>'
    template=template.replace('<canvas id="view"',fallback+'<canvas id="view"')
    template=template.replace('<button ','<button disabled ').replace('<input ','<input disabled ').replace('<select ','<select disabled ')
    boot='''
    window.showViewerFallback=function(message){
      const canvas=document.getElementById('view');
      canvas.style.visibility='hidden';canvas.dataset.rendered='false';
      document.getElementById('fallbackImage').hidden=false;
      const status=document.getElementById('loadStatus');status.hidden=false;
      status.textContent='三维预览未能启动，已显示装配图。'+(message?' 原因：'+message:'');
      document.querySelectorAll('button,input,select').forEach(el=>el.disabled=true);
      document.documentElement.dataset.viewerMode='static';
    };
    window.viewerRenderSucceeded=function(){
      document.getElementById('view').style.visibility='visible';
      document.getElementById('fallbackImage').hidden=true;
      document.getElementById('loadStatus').hidden=true;
      document.querySelectorAll('button,input,select').forEach(el=>el.disabled=false);
      document.documentElement.dataset.viewerMode='3d';
    };
    window.addEventListener('error',event=>window.showViewerFallback(event.message||'脚本执行失败'));
    window.addEventListener('unhandledrejection',event=>window.showViewerFallback(String(event.reason||'加载失败')));
    '''
    vendor=(out/'preview/three.min.js').read_text(encoding='utf-8')
    vendor=re.sub(r'</script',r'<\\/script',vendor,flags=re.I)
    source=(root/'viewer.js').read_text(encoding='utf-8')
    source=source.replace('__SCENE_DATA__',json.dumps(scene,separators=(',',':'))).replace('__PIVOT__',str(p['pivot_height_above_deck']))
    wrapped="try{(()=>{\n"+source+"\n})();}catch(error){window.showViewerFallback(error.message||String(error));}"
    html=template+'<noscript><p>浏览器未执行 JavaScript，当前显示装配图；三维交互需在支持脚本的浏览器中打开。</p></noscript><script>'+boot+'</script><script>'+vendor+'</script><script>'+wrapped+'</script></body></html>'
    path=out/'preview/assembly_viewer.html';path.write_text(html,encoding='utf-8')
    (root.parent/'pendulum_新底盘_独立预览.html').write_text(html,encoding='utf-8')
    print(f'STANDALONE_VIEWER {path} ({path.stat().st_size} bytes)',flush=True)

if __name__=='__main__':
    root=Path(__file__).resolve().parent;out=root/'output'
    build(root,out,json.loads((out/'preview/scene.json').read_text(encoding='utf-8')),json.loads((root/'parameters.json').read_text(encoding='utf-8')))
