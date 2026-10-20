(function(root){'use strict';
const numeric=/^-?\d+(?:\.\d+)?$/;
const sizes={Q:[9],T:[11,16],B:[25],U:[17,18],S:[22],D:[49],M:[21],V:[14,15,16],E:[19],P:[9],Y:[8,10],H:[6],O:[5],W:[18],F:[10]};
const stateNames=['已停止','最低点标定','等待顶端','顶端捕捉中','直立运行','已退出','自动起摆'];
const reasonNames=['无','STOP','角度越界','传感器无效','控制超时','零点标定','模式冲突','参数修改','轮速超限','运行超时','行程越界'];
const uint=(v,max=4294967295)=>Number.isInteger(v)&&v>=0&&v<=max;
const bit=v=>v===0||v===1;
function parseLine(line){
 line=line.trim();if(!line)return null;
 if(line.startsWith('#')){
  if(!/^#F103_(?:BAL_TRACE_V\d+|SWING_TRACE_V\d+|PID_CSV_V\d+)(?:,|$)/.test(line))return null;
  const fw=line.match(/(?:,fw=)(0x[\da-f]+)(?:,|$)/i);
  return {kind:'schema',text:line,fw:fw?fw[1]:null};
 }
 const p=line.split(','),kind=p[0];
 if(kind==='A'&&p.length>=4&&p.length<=5&&/^\d+$/.test(p[1])&&/^(OK|ERR)$/.test(p[2])&&p.slice(3).every(v=>/^[A-Z0-9_]+$/.test(v)))return {kind,text:line,ok:p[2]==='OK'};
 if(kind==='C'){
  if(p.length!==28||!/^0x[\da-f]+$/i.test(p[2])||!p.slice(1).filter((_,i)=>i!==1).every(v=>numeric.test(v)&&Number.isFinite(Number(v)))||!uint(Number(p[1])))return null;
  return {kind,ms:Number(p[1]),fw:p[2]};
 }
 if(kind==='X'){
  if(p.length!==11||!/^\d+$/.test(p[1])||!uint(Number(p[1]))||! /^(NONE|STOP|ANGLE|SENSOR|TIMING|ZERO|CONFLICT|CONFIG|SPEED|RUN_TIME|TRAVEL|UNKNOWN)$/.test(p[2])||!p.slice(3).every(v=>numeric.test(v)&&Number.isFinite(Number(v))))return null;
  const n=p.slice(3).map(Number);if(!uint(n[0],10))return null;
  return {kind,ms:Number(p[1]),reasonText:p[2],reason:n[0],peakRpm:n[1],maxRpm:n[2],overspeedMs:n[3],dwellMs:n[4],rawErrorDeg:n[5],runMs:n[6],coalesced:n[7],state:5};
 }
 if(!sizes[kind]?.includes(p.length)||!p.slice(1).every(v=>numeric.test(v)&&Number.isFinite(Number(v))))return null;
 const n=p.slice(1).map(Number);if(!uint(n[0]))return null;
 if(kind==='Q'){
  if(!uint(n[1],4095)||n[2]<0||n[2]>=360||!uint(n[3],255)||!n.slice(4).every(bit))return null;
  if(n[5]!==((n[3]&32)?1:0)||n[6]!==((n[3]&16)?1:0)||n[7]!==((n[3]&8)?1:0)||n[4]&&(!n[5]||n[6]||n[7]))return null;
  return {kind,ms:n[0],raw:n[1],deg:n[2],status:n[3],valid:!!n[4],md:!!n[5],ml:!!n[6],mh:!!n[7]};
 }
 if(kind==='T'){
  if(!uint(n[1],4095)||!uint(n[8],6)||!uint(n[9],10)||Math.abs(n[2])>180)return null;
  return {kind,ms:n[0],raw:n[1],deg:n[1]*360/4096,errorDeg:n[2],rate:n[3],output:n[4],meanRpm:n[5],refDeg:n[6],guardRpm:n[7],state:n[8],reason:n[9],fastRpm:n.length===15?n.slice(10,14):null,robustRpm:n[14]??null};
 }
 if(kind==='B'){
  if(!uint(n[1],6)||!uint(n[2],10)||!bit(n[3])||!bit(n[4])||!uint(n[7],4095)||!uint(n[8],4095))return null;
  return {kind,ms:n[0],state:n[1],reason:n[2],calibrated:!!n[3],valid:!!n[4],bottomRaw:n[7],uprightRaw:n[8],bottomDeg:n[9],errorDeg:n[10],rate:n[11],output:n[12]};
 }
 if(kind==='U'){
  if(!uint(n[1],4)||n[2]<0||n[2]>=360)return null;
  // U reports a calibrated bottom-relative angle, not AS5600 raw/STATUS.
  return {kind,ms:n[0],phase:n[1],bottomDeg:n[2],rate:n[3],energy:n[4],targetEnergy:n[5],output:n[6],position:n[7],meanRpm:n[8],limit:n[9],travelRev:n[10],runMs:n[11],maxRunMs:n[12],omega2:n[13],gain:n[14],captureRate:n[15],captureDeg:n[16]??null,state:n[1]===3?4:n[1]===4?5:6};
 }
 if(kind==='S')return {kind,ms:n[0],readOk:n[19],errors:n[20],stack:n[15]};
 if(kind==='D')return {kind,ms:n[0],mode:n[3],missed:n[8]};
 if(kind==='M')return {kind,ms:n[0],dmaErrors:n[5],dmaLate:n[6]};
 return {kind,ms:n[0],values:n.slice(1)};
}
function parseText(text){
 // Serial assistants may wrap any numeric field across timestamped receive chunks.
 const clean=text.replace(/\[\d{2}:\d{2}:\d{2}\s+\d{3}\]发[:：][\s\S]*?(?=\[\d{2}:\d{2}:\d{2}\s+\d{3}\][收发][:：]|$)/g,'').replace(/\[\d{2}:\d{2}:\d{2}\s+\d{3}\][收发][:：]/g,'');
 const markers=[...clean.matchAll(/#[^\r\n]*|[A-Z],(?=\d)/g)].filter(m=>m[0][0]==='#'||m.index===0||/[\s\d]/.test(clean[m.index-1])),events=[];
 for(let i=0;i<markers.length;i++){
  const m=markers[i];
  if(m[0][0]!=='#'&&m.index>0&&!/[\s\d]/.test(clean[m.index-1]))continue;
  const line=m[0][0]==='#'?m[0]:clean.slice(m.index,markers[i+1]?.index??clean.length).replace(/[\r\n]/g,'').trim();
  const p=parseLine(line);if(p)events.push(p);
 }
 return events;
}
class Framer{
 constructor(onPacket){this.onPacket=onPacket;this.buf='';this.dropping=false;this.bad=0;}
 feed(text){for(const ch of text){if(ch==='\r'||ch==='\n'){if(!this.dropping&&this.buf){const p=parseLine(this.buf);if(p)this.onPacket(p);else if(/^[A-Z],/.test(this.buf))this.bad++;}this.buf='';this.dropping=false;}else if(!this.dropping){this.buf+=ch;if(this.buf.length>8192){this.bad++;this.buf='';this.dropping=true;}}}}
}
class Samples{
 constructor(limit=50000){this.limit=limit;this.reset();}
 reset(){this.rows=[];this.validCount=0;this.knownCount=0;this.offset=0;this.lastRaw=null;this.lastT=0;this.interval=100;this.boot=0;this.base=null;}
 add(p){
  let gap=false;const same=this.lastRaw===p.ms&&this.rows.length>0;
  const prior=same?this.rows.pop():null;
  if(prior){if(prior.valid===true)this.validCount--;if(prior.valid!==null)this.knownCount--;}
  if(this.lastRaw!==null&&p.ms<this.lastRaw){if(this.lastRaw>0xffff0000&&p.ms<65536)this.offset+=4294967296;else{this.offset=this.lastT+this.interval-p.ms;this.boot++;gap=true;}}
  const t=p.ms+this.offset;if(this.base===null)this.base=t;
  const dt=this.lastRaw===null?0:t-this.lastT;if(dt>Math.max(1000,this.interval*4))gap=true;
  const data={raw:null,deg:null,status:null,valid:null,md:null,ml:null,mh:null,errorDeg:null,bottomDeg:null,rate:null,output:null,state:null,reason:null,...prior,...p};
  const last=this.rows.at(-1);let unwrapped=data.deg;
  if(last&&data.deg!==null&&last.deg!==null&&!gap){let d=data.deg-last.deg;if(d>180)d-=360;if(d<-180)d+=360;unwrapped=last.unwrapped+d;}
  if(dt>0&&dt<1000)this.interval=dt;
  const sources=[...new Set([...(prior?.sources.split('+')??[]),p.kind])].join('+');
  const row={...data,sources,t,elapsed:(t-this.base)/1000,unwrapped,gap:gap||!!prior?.gap,boot:this.boot};
  this.rows.push(row);if(row.valid===true)this.validCount++;if(row.valid!==null)this.knownCount++;
  if(this.rows.length>this.limit){const gone=this.rows.shift();if(gone.valid===true)this.validCount--;if(gone.valid!==null)this.knownCount--;}
  this.lastRaw=p.ms;this.lastT=t;return row;
 }
}
function series(rows,mode='auto',unwrap=false){
 if(mode==='auto'){
  const last=rows.at(-1);
  mode=last?.state===6&&last.bottomDeg!==null?'bottom':Number.isFinite(last?.errorDeg)?'error':Number.isFinite(last?.deg)?'mechanical':Number.isFinite(last?.bottomDeg)?'bottom':'mechanical';
 }
 const field=mode==='mechanical'?(unwrap?'unwrapped':'deg'):mode==='error'?'errorDeg':'bottomDeg';
 return {mode,label:mode==='mechanical'?'机械角度':mode==='error'?'顶端偏角':'最低点角度',rows:rows.filter(p=>Number.isFinite(p[field])).map(p=>({...p,value:p[field]}))};
}
function csv(rows){
 const columns=['elapsed','ms','boot','raw','deg','unwrapped','status','valid','md','ml','mh','errorDeg','bottomDeg','rate','output','state','reason','sources'];
 return '\ufeff'+'elapsed_s,mcu_ms,boot,raw,mechanical_deg,unwrapped_deg,status,valid,md,ml,mh,upright_error_deg,bottom_deg,rate_rad_s,output,state,reason,source\r\n'+rows.map(p=>columns.map(k=>p[k]===null||p[k]===undefined?'':typeof p[k]==='boolean'?+p[k]:p[k]).join(',')).join('\r\n');
}
root.SerialTelemetry={parseLine,parseText,Framer,Samples,series,csv,stateNames,reasonNames};
})(typeof window!=='undefined'?window:globalThis);
