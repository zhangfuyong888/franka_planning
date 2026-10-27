// 在无浏览器环境验证应用脚本的数据映射和交互；不代替真实 WebGL 显示验收。
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const text=fs.readFileSync(process.argv[2],'utf8');const scripts=[...text.matchAll(/<script>([\s\S]*?)<\/script>/g)];
assert(scripts.length===2);assert(text.includes('plotly.js (gl3d - minified) v3.1.0'));assert(!text.includes('<script src='));
const elements=new Map(),calls=[];
function element(id=''){
 let val='',html='';const e={id,checked:true,max:25,min:0,dataset:{},clientWidth:500,textContent:'',children:[],appendChild(o){this.children.push(o);if(id==='group'&&!val)val=o.value},getContext(){return {scale(){},clearRect(){},fillText(){},strokeRect(){},beginPath(){},moveTo(){},lineTo(){},stroke(){}}}};
 Object.defineProperty(e,'value',{get(){return val},set(v){val=String(v)}});
 Object.defineProperty(e,'innerHTML',{get(){return html},set(v){html=v;for(let match of v.matchAll(/id="([^"]+)"/g))get(match[1])}});
 return e;
}
function get(id){if(!elements.has(id))elements.set(id,element(id));return elements.get(id)}
let nextFrame;const context={console,performance:{now:()=>1000},window:{addEventListener(){},devicePixelRatio:1},document:{getElementById:get,createElement:()=>element(),body:element('body')},Plotly:{async react(id,traces,layout){assert(traces.length>=4);for(let t of traces)assert.equal(t.x.length,t.y.length);assert.deepEqual(layout.scene.xaxis.range,[-.9,.9]);calls.push({id,traces})}},requestAnimationFrame(fn){nextFrame=fn}};
context.setTimeout=setTimeout;vm.createContext(context);vm.runInContext(scripts[1][1],context);
(async()=>{
 await new Promise(r=>setImmediate(r));await vm.runInContext('group()',context);assert(calls.length>0);let before=calls.length;
 const testTime=Math.min(1.2,+get('time').max/4);get('time').value=testTime;await get('time').oninput();assert(calls.length>before);assert.equal(get('timevalue').textContent,testTime.toFixed(2));
 await get('step').onclick();await new Promise(r=>setImmediate(r));const stepTime=Math.min(+get('time').max,testTime+.05);assert(Math.abs(+get('time').value-stepTime)<.001);
 get('paths').checked=false;await get('paths').oninput();assert(!calls.at(-1).traces.some(x=>x.name==='处理后路径'));
 get('paths').checked=true;get('plantime').value=2.;await get('plantime').oninput();
 get('play').onclick();assert.equal(get('play').textContent,'暂停');nextFrame(1200);await new Promise(r=>setImmediate(r));assert(+get('time').value>stepTime);get('play').onclick();
 for(let option of get('group').children){get('group').value=option.value;await get('group').onchange()}
 get('plantime').value=0;await get('plantime').oninput();get('planplay').onclick();assert.equal(get('planplay').textContent,'暂停规划快照');nextFrame(1300);await new Promise(r=>setImmediate(r));assert(+get('plantime').value>0);get('planplay').onclick();
 assert(get('table').innerHTML.includes('planning_time'));console.log(JSON.stringify({status:'passed',plot_calls:calls.length,groups:get('group').children.length,verification:'Node VM 数据与交互检查，未验证浏览器/WebGL显示'}));
})().catch(e=>{console.error(e);process.exitCode=1});
