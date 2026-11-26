(function app(){
"use strict";
const $=id=>document.getElementById(id), clone=x=>JSON.parse(JSON.stringify(x)), canvas=$("canvas"),ctx=canvas.getContext("2d"),stage=$("stage");
const library=JSON.parse($("catalog").textContent);
let doc=null,c=null,mode="library",tool="select",selected=null,inspect=0,overlay="",source=null,field=null,history=[],future=[],dirty=false,space=false,drag=null,cam={s:1,x:0,y:0};
let viewport={w:1,h:1};
const palette=["#2a78d6","#1baf7a","#e39b0b","#8a5cd6"];
const color=a=>palette[(a.size-1)/2]??"hsl("+(((a.size-9)/2*137.508+18)%360)+",58%,45%)", index=(x,y)=>y*doc.grid[0].length+x, active=()=>doc?.agents.find(a=>a.id===selected), size=()=>active()?.size||inspect;
function status(s="",error=false){$("status").textContent=s;$("status").classList.toggle("error",error)}
function guard(fn){try{fn()}catch(e){status(e.message,true)}}
function snapshot(){return clone(doc)}
function remember(old){history.push(old);if(history.length>80)history.shift();future=[];dirty=true}
function refresh(){c=Geo.clearance(doc.grid);field=null;if(source!==null&&source>=c.length)source=null;panel();draw()}
function resetSelection(){selected=null;inspect=0;overlay="";source=null;field=null}
function switchMode(m){mode=m;$("library").hidden=m!=="library";$("workspace").hidden=m==="library";$("editor").hidden=m!=="edit";$("historyControls").hidden=m!=="edit";$("downloadWrap").hidden=m==="library"||!doc;closeDownload();for(const t of ["library","view","edit"])$(t+"Tab").classList.toggle("active",m===t);if(m==="library")renderLibrary();else{if(!doc){switchMode("library");return}tool="select";panel();requestAnimationFrame(()=>{resize();if(cam.s===1)fit()})}}
function open(entry,m="view"){if(dirty&&!confirm("Несохранённые изменения будут потеряны. Продолжить?"))return;doc=clone(entry);Geo.validate(doc.grid,doc.agents);history=[];future=[];dirty=false;resetSelection();cam={s:1,x:0,y:0};refresh();switchMode(m);requestAnimationFrame(fit);status("Карта открыта")}
function renderLibrary(){
 const q=$("search").value.toLowerCase(),root=$("cards");root.replaceChildren();
 for(const entry of library.filter(e=>e.name.toLowerCase().includes(q))){
  const card=document.createElement("article");card.className="mapcard";const cv=document.createElement("canvas");cv.width=240;cv.height=125;const cx=cv.getContext("2d");const [w,h]=Geo.dims(entry.grid),s=Math.min(240/w,125/h),ox=(240-w*s)/2,oy=(125-h*s)/2;
  cx.fillStyle="#fff";cx.fillRect(ox,oy,w*s,h*s);cx.fillStyle="#384253";let free=0;for(let y=0;y<h;y++)for(let x=0;x<w;x++){if(entry.grid[y][x]==="#")cx.fillRect(ox+x*s,oy+y*s,Math.ceil(s),Math.ceil(s));else free++}
  const title=document.createElement("strong");title.textContent=entry.name;const meta=document.createElement("div");meta.className="muted";meta.textContent=w+" × "+h+" · "+((1-free/(w*h))*100).toFixed(1)+"% стен"+(entry.agents.length?" · "+entry.agents.length+" аг.":"");const actions=document.createElement("div");actions.className="buttons";for(const [m,label] of [["view","Открыть"],["edit","Редактировать"]]){const b=document.createElement("button");b.textContent=label;b.onclick=e=>{e.stopPropagation();guard(()=>open(entry,m))};actions.append(b)}const remove=document.createElement("button");remove.className="icon-button remove-map";remove.innerHTML="<svg viewBox=\"0 0 24 24\" width=\"20\" height=\"20\" fill=\"none\" stroke=\"currentColor\" stroke-width=\"1.8\" stroke-linecap=\"round\" stroke-linejoin=\"round\" aria-hidden=\"true\"><path d=\"M4 7h16M9 7V4h6v3M6 7l1 13h10l1-13M10 10v6m4-6v6\"/></svg>";remove.title="Удалить из каталога сессии";remove.setAttribute("aria-label","Удалить из каталога сессии: "+entry.name);remove.onclick=e=>{e.stopPropagation();const i=library.indexOf(entry);if(i>=0){library.splice(i,1);renderLibrary();status("Удалено из временного каталога: "+entry.name)}};actions.append(remove);card.append(cv,title,meta,actions);card.onclick=()=>guard(()=>open(entry));root.append(card);
 }
 if(!root.children.length)root.textContent="Нет подходящих карт. Импортируйте файл или создайте новую.";
}
function panel(){
 if(!doc)return;const [w,h]=Geo.dims(doc.grid),free=c?c.reduce((n,v)=>n+(v>0),0):0;
 $("mapName").textContent=doc.name+(dirty?" *":"");$("stats").textContent=w+" × "+h+" · свободно "+free+" / "+w*h+" · агентов "+doc.agents.length;
 if(document.activeElement!==$("name"))$("name").value=doc.name;
 const a=active(),s=size(),sel=$("inspectSize"),values=[0,1,3,5,7,9,...(s>9?[s]:[])];
 sel.replaceChildren(...values.map(v=>{let o=document.createElement("option");o.value=v;o.textContent=v?v+" × "+v:"Выключена";o.disabled=v>9;return o}));sel.value=String(s);
 const root=$("agents");root.replaceChildren();for(const a of doc.agents){let b=document.createElement("button");b.classList.toggle("active",a.id===selected);b.style.borderLeft="4px solid "+color(a);b.textContent="#"+a.id+" · "+a.size+"×"+a.size+" · ("+a.start.join(", ")+")"+(a.goal?" → ("+a.goal.join(", ")+")":"");b.onclick=()=>choose(a.id);root.append(b)}
 if(!doc.agents.length)root.textContent="Агентов нет";
 for(const b of $("tools").querySelectorAll("button[data-tool]"))b.classList.toggle("active",b.dataset.tool===tool);
 $("undo").disabled=!history.length;$("redo").disabled=!future.length;$("deleteAgent").disabled=!a;$("clearGoal").disabled=!a||!a.goal;
 $("components").classList.toggle("active",overlay==="components");$("distances").classList.toggle("active",overlay==="distance");
 $("selection").textContent=s?(a?"Агент #"+a.id+" · ":"")+s+"×"+s+"\nДопустимые центры: "+c.reduce((n,v)=>n+(v>=s),0)+" / "+free:"Клик по агенту — его статическая эрозия.";
 if(s&&overlay==="components"){let f=getField();$("selection").textContent+="\nКомпонент: "+f.counts.length+"\nКрупнейшая: "+f.counts.reduce((m,n)=>Math.max(m,n),0)+"\nРазмеры: "+f.counts.slice().sort((a,b)=>b-a).slice(0,12).join(", ")+(f.counts.length>12?" …":"")}
 if(overlay==="distance")$("selection").textContent+="\n"+(source===null?"Кликните по допустимой клетке.":"Расстояние в шагах от ("+source%w+", "+Math.floor(source/w)+"). Серые клетки недостижимы.");
}
function choose(id){selected=selected===id?null:id;inspect=0;overlay="";source=null;field=null;panel();draw()}
function setSize(s){if(s&&!Geo.validSize(s))throw Error("Сторона должна быть положительным нечётным числом");selected=null;inspect=s;overlay="";source=null;field=null;panel();draw()}
function getField(){if(!field&&size()&&overlay)field=Geo.flood(doc.grid,c,size(),overlay==="distance"?source:null);return field}
function resize(){if(mode==="library")return;const rect=stage.getBoundingClientRect(),dpr=window.devicePixelRatio||1;viewport={w:rect.width,h:rect.height};canvas.width=Math.max(1,Math.round(rect.width*dpr));canvas.height=Math.max(1,Math.round(rect.height*dpr));ctx.setTransform(dpr,0,0,dpr,0,0);draw()}
function fit(){if(!doc||mode==="library")return;const [w,h]=Geo.dims(doc.grid);cam.s=Math.max(.05,Math.min((viewport.w-36)/w,(viewport.h-36)/h,75));cam.x=(viewport.w-w*cam.s)/2;cam.y=(viewport.h-h*cam.s)/2;draw()}
function zoom(f,x=viewport.w/2,y=viewport.h/2){const s=Math.max(.05,Math.min(180,cam.s*f));cam.x=x-(x-cam.x)*s/cam.s;cam.y=y-(y-cam.y)*s/cam.s;cam.s=s;draw()}
function draw(){
 if(!doc||mode==="library")return;const [w,h]=Geo.dims(doc.grid),s=cam.s,sz=size(),f=overlay==="distance"&&source===null?null:getField();
 ctx.clearRect(0,0,viewport.w,viewport.h);ctx.save();ctx.translate(cam.x,cam.y);ctx.scale(s,s);
 const x0=Math.max(0,Math.floor(-cam.x/s)),y0=Math.max(0,Math.floor(-cam.y/s)),x1=Math.min(w,Math.ceil((viewport.w-cam.x)/s)),y1=Math.min(h,Math.ceil((viewport.h-cam.y)/s));
 let maxD=1;if(f&&overlay==="distance")for(const d of f.labels)maxD=Math.max(maxD,d);
 ctx.fillStyle="#fbfcfd";ctx.fillRect(0,0,w,h);
 for(let y=y0;y<y1;y++)for(let x=x0;x<x1;x++){let i=y*w+x,v=c[i],fill=null;if(!v)fill="#384253";else if(sz&&v<sz)fill="#c9d1db";else if(f&&overlay==="components")fill="hsl("+((f.labels[i]*137.508)%360)+",55%,79%)";else if(f&&overlay==="distance")fill=f.labels[i]<0?"#c9d1db":"hsl(215,72%,"+(95-53*f.labels[i]/maxD)+"%)";else if(sz)fill="#eff8f1";if(fill){ctx.fillStyle=fill;ctx.fillRect(x,y,1,1)}}
 if(s>=9){ctx.beginPath();ctx.strokeStyle="#7b889b44";ctx.lineWidth=1/s;for(let x=x0;x<=x1;x++){ctx.moveTo(x,y0);ctx.lineTo(x,y1)}for(let y=y0;y<=y1;y++){ctx.moveTo(x0,y);ctx.lineTo(x1,y)}ctx.stroke()}
 if(f&&overlay==="distance"&&s>=28){ctx.font=(11/s)+"px system-ui";ctx.textAlign="center";ctx.textBaseline="middle";for(let y=y0;y<y1;y++)for(let x=x0;x<x1;x++){let d=f.labels[y*w+x];if(d>=0){ctx.fillStyle=d/maxD>.62?"#fff":"#26364c";ctx.fillText(d,x+.5,y+.5)}}}
 for(const a of doc.agents)if(a.goal){let [x,y]=a.goal,k=a.size/2;ctx.strokeStyle=color(a);ctx.lineWidth=2/s;ctx.setLineDash([4/s,3/s]);ctx.strokeRect(x+.5-k+.08,y+.5-k+.08,a.size-.16,a.size-.16);ctx.setLineDash([]);ctx.beginPath();ctx.arc(x+.5,y+.5,.2,0,Math.PI*2);ctx.stroke()}
 for(const a of doc.agents){let [x,y]=a.start,k=a.size/2;ctx.globalAlpha=.88;ctx.fillStyle=color(a);ctx.fillRect(x+.5-k+.05,y+.5-k+.05,a.size-.1,a.size-.1);ctx.globalAlpha=1;ctx.strokeStyle=selected===a.id?"#172337":color(a);ctx.lineWidth=(selected===a.id?3:1)/s;ctx.strokeRect(x+.5-k+.05,y+.5-k+.05,a.size-.1,a.size-.1);if(s*a.size>=24){ctx.fillStyle="#fff";ctx.textAlign="center";ctx.textBaseline="middle";ctx.font=Math.min(.65*a.size,14/s)+"px system-ui";ctx.fillText("#"+a.id,x+.5,y+.5)}}
 if(source!==null&&overlay==="distance"){ctx.strokeStyle="#e35c2d";ctx.lineWidth=2/s;ctx.strokeRect(source%w,Math.floor(source/w),1,1)}
 if(drag?.kind==="rect"&&drag.end){let [x,y]=drag.start,[xx,yy]=drag.end;ctx.fillStyle="#38425355";ctx.fillRect(Math.min(x,xx),Math.min(y,yy),Math.abs(x-xx)+1,Math.abs(y-yy)+1)}
 ctx.restore();
}
function position(e){const r=canvas.getBoundingClientRect(),px=e.clientX-r.left,py=e.clientY-r.top;return {px,py,p:[Math.floor((px-cam.x)/cam.s),Math.floor((py-cam.y)/cam.s)]}}
function inside(p){return p[0]>=0&&p[1]>=0&&p[0]<doc.grid[0].length&&p[1]<doc.grid.length}
function hit(p){return doc.agents.find(a=>Math.abs(p[0]-a.start[0])<a.size/2&&Math.abs(p[1]-a.start[1])<a.size/2)}
function change(fn){let old=snapshot();try{fn();Geo.validate(doc.grid,doc.agents);remember(old);refresh();status("Изменено")}catch(e){doc=old;refresh();status(e.message,true)}}
function paint(p,wall){if(!inside(p))return;let [x,y]=p,v=wall?"#":".";if(doc.grid[y][x]===v)return;if(wall&&doc.agents.some(a=>[a.start,a.goal].some(q=>q&&Math.abs(q[0]-x)<a.size/2&&Math.abs(q[1]-y)<a.size/2))){status("Клетка занята агентом или его целью",true);return}doc.grid[y]=doc.grid[y].slice(0,x)+v+doc.grid[y].slice(x+1);drag.changed=true}
function line(a,b,wall){let [x,y]=a,[xx,yy]=b,dx=Math.abs(xx-x),dy=-Math.abs(yy-y),sx=x<xx?1:-1,sy=y<yy?1:-1,err=dx+dy;for(let j=0;j<10000;j++){paint([x,y],wall);if(x===xx&&y===yy)break;let e=2*err;if(e>=dy){err+=dy;x+=sx}if(e<=dx){err+=dx;y+=sy}}}
function click(p){
 if(!inside(p))return;
 if(overlay==="distance"&&(mode!=="edit"||tool==="select")){if(c[index(...p)]<size()){status("Недопустимый источник",true);return}source=index(...p);field=null;panel();draw();return}
 let a=hit(p);if(mode!=="edit"||tool==="select"){if(a)choose(a.id);else{resetSelection();panel();draw()}return}
 if(tool==="agent"){let s=Number($("size").value);change(()=>{let id=doc.agents.reduce((m,a)=>Math.max(m,a.id),-1)+1;doc.agents.push({id,size:s,start:p,goal:null});selected=id;inspect=0;overlay="";source=null})}
 if(tool==="move"||tool==="goal"){if(!active()){if(a)choose(a.id);else status("Сначала выберите агента",true);return}change(()=>{active()[tool==="move"?"start":"goal"]=p})}
}
canvas.addEventListener("contextmenu",e=>e.preventDefault());
canvas.addEventListener("wheel",e=>{e.preventDefault();let {px,py}=position(e);zoom(Math.exp(-e.deltaY*.0015),px,py)},{passive:false});
canvas.addEventListener("pointerdown",e=>{if(!doc)return;const {p,px,py}=position(e);if(e.button!==0&&e.button!==1&&e.button!==2)return;e.preventDefault();canvas.setPointerCapture(e.pointerId);
 if(e.button!==0||space){drag={kind:"pan",px,py,x:cam.x,y:cam.y};return}
 if(mode==="edit"&&inside(p)&&["wall","erase","rect"].includes(tool)){drag={kind:tool==="rect"?"rect":"paint",start:p,end:p,old:snapshot(),changed:false,wall:tool!=="erase"};if(drag.kind==="paint"){paint(p,drag.wall);refresh()}return}
 drag={kind:"click",start:p,px,py,x:cam.x,y:cam.y};
});
canvas.addEventListener("pointermove",e=>{if(!doc)return;const {p,px,py}=position(e);$("coords").textContent=inside(p)?"Клетка ("+p.join(", ")+") · clearance "+c[index(...p)]:"Клетка —";
 if(!drag)return;if(drag.kind==="click"&&Math.hypot(px-drag.px,py-drag.py)>5)drag.kind="pan";
 if(drag.kind==="pan"){cam.x=drag.x+px-drag.px;cam.y=drag.y+py-drag.py;draw()}
 if(drag.kind==="paint"&&inside(p)){line(drag.end,p,drag.wall);drag.end=p;refresh()}
 if(drag.kind==="rect"){drag.end=[Math.max(0,Math.min(doc.grid[0].length-1,p[0])),Math.max(0,Math.min(doc.grid.length-1,p[1]))];draw()}
});
function finish(e,cancel=false){if(!drag)return;let d=drag;drag=null;if(cancel){if(d.old){doc=d.old;refresh()}return}
 if(d.kind==="click"){click(position(e).p);return}
 if(d.kind==="rect"){drag=d;let [x,y]=d.start,[xx,yy]=d.end;for(let j=Math.min(y,yy);j<=Math.max(y,yy);j++)for(let i=Math.min(x,xx);i<=Math.max(x,xx);i++)paint([i,j],true);drag=null}
 if(d.changed){remember(d.old);refresh()}else draw()
}
canvas.addEventListener("pointerup",e=>finish(e));canvas.addEventListener("pointercancel",e=>finish(e,true));
window.addEventListener("keydown",e=>{if(e.key==="Escape"&&!$("downloadMenu").hidden){closeDownload();$("downloadToggle").focus();return}if(["INPUT","SELECT","TEXTAREA"].includes(e.target.tagName)||e.target.isContentEditable)return;if(e.code==="Space"){space=true;e.preventDefault()}if(e.key==="Escape"&&doc){resetSelection();tool="select";panel();draw()}if(e.key.toLowerCase()==="f")fit();if((e.ctrlKey||e.metaKey)&&!e.altKey&&mode==="edit"&&(e.code==="KeyZ"||e.key.toLowerCase()==="z")){e.preventDefault();if(!drag)restore(e.shiftKey)}});window.addEventListener("keyup",e=>{if(e.code==="Space")space=false});window.addEventListener("blur",()=>space=false);new ResizeObserver(resize).observe(stage);
function restore(redo){let from=redo?future:history,to=redo?history:future;if(!from.length)return;to.push(snapshot());doc=from.pop();resetSelection();dirty=true;refresh()}
function download(name,text,type){let url=URL.createObjectURL(new Blob([text],{type})),a=document.createElement("a");a.href=url;a.download=name;document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);status("Файл передан браузеру для скачивания")}
function baseName(){return doc.name.split(/[\\/]/).pop().replace(/\.(grid|map|json)$/i,"").replace(/[<>:"|?*]/g,"_")||"map"}
function parseFile(name,text){if(/\.json$/i.test(name)){let obj=JSON.parse(text);if(obj.version!==1)throw Error("Поддерживается version: 1");let grid;if(Array.isArray(obj.grid))grid=Geo.parseGrid(obj.grid.join("\n"));else if(typeof obj.map==="string"){let dir=name.includes("/")?name.slice(0,name.lastIndexOf("/")+1):"",matches=library.filter(e=>e.name===dir+obj.map||e.name===obj.map);if(matches.length!==1)throw Error("Не найдена однозначная карта: "+obj.map);grid=matches[0].grid.slice()}else throw Error("Требуется grid или map");Geo.validate(grid,obj.agents);return {name,grid,agents:obj.agents.map(a=>({id:a.id,size:a.size,start:a.start,goal:a.goal}))}}return {name,grid:Geo.parseGrid(text),agents:[]}}
async function importFiles(list){let files=[...list].filter(f=>/\.(grid|map|json)$/i.test(f.name)).sort((a,b)=>Number(/\.json$/i.test(a.name))-Number(/\.json$/i.test(b.name))),entries=[],errors=[];for(const f of files){try{let e=parseFile(f.webkitRelativePath||f.name,await f.text());library.push(e);entries.push(e)}catch(e){errors.push(f.name+": "+e.message)}}renderLibrary();if(entries.length===1)guard(()=>open(entries[0]));else switchMode("library");status(errors.length?errors.join("; "):"Импортировано: "+entries.length,!!errors.length)}
$("files").onchange=e=>{importFiles(e.target.files);e.target.value=""};$("directory").onchange=e=>{importFiles(e.target.files);e.target.value=""};
$("open").onclick=()=>$("files").click();$("folder").onclick=()=>$("directory").click();
$("new").onclick=()=>guard(()=>{let a=prompt("Ширина и высота (например 24 16)","24 16");if(a===null)return;let [w,h]=a.trim().split(/\s+/).map(Number);if(!Number.isInteger(w)||!Number.isInteger(h)||w<1||h<1||w*h>1000000)throw Error("Некорректные размеры");open({name:"new.grid",grid:Array(h).fill(".".repeat(w)),agents:[]},"edit")});
$("libraryTab").onclick=()=>switchMode("library");$("viewTab").onclick=()=>switchMode("view");$("editTab").onclick=()=>switchMode("edit");$("search").oninput=renderLibrary;
$("fit").onclick=fit;$("minus").onclick=()=>zoom(.8);$("plus").onclick=()=>zoom(1.25);$("undo").onclick=()=>restore(false);$("redo").onclick=()=>restore(true);
$("tools").onclick=e=>{if(e.target.dataset.tool){tool=e.target.dataset.tool;overlay="";source=null;field=null;panel();draw()}};
$("name").onchange=()=>{let n=$("name").value.trim();if(n&&n!==doc.name)change(()=>doc.name=n);else panel()};
$("deleteAgent").onclick=()=>{if(active())change(()=>{doc.agents=doc.agents.filter(a=>a.id!==selected);resetSelection()})};
$("clearGoal").onclick=()=>{if(active())change(()=>active().goal=null)};
$("inspectSize").onchange=e=>guard(()=>setSize(Number(e.target.value)));
$("components").onclick=()=>{if(!size())inspect=1;overlay=overlay==="components"?"":"components";source=null;field=null;tool="select";panel();draw()};
$("distances").onclick=()=>{if(!size())inspect=1;overlay=overlay==="distance"?"":"distance";source=null;field=null;tool="select";panel();draw()};
$("reset").onclick=()=>{resetSelection();panel();draw()};
function closeDownload(){ $("downloadMenu").hidden=true;$("downloadToggle").setAttribute("aria-expanded","false") }
$("downloadToggle").onclick=()=>{const show=$("downloadMenu").hidden;$("downloadMenu").hidden=!show;$("downloadToggle").setAttribute("aria-expanded",String(show));if(show)$("saveMap").focus()};
document.addEventListener("click",e=>{if(!$("downloadWrap").contains(e.target))closeDownload()});
$("saveMap").onclick=()=>{closeDownload();download(baseName()+".grid",doc.grid.join("\n")+"\n","text/plain;charset=utf-8")};
$("saveCase").onclick=()=>{closeDownload();download(baseName()+".json",JSON.stringify({version:1,grid:doc.grid,agents:doc.agents},null,2)+"\n","application/json")};
$("addLibrary").onclick=()=>{library.push(snapshot());dirty=false;panel();status("Добавлено в каталог текущей сессии")};
window.addEventListener("dragover",e=>{if(e.dataTransfer.types.includes("Files"))e.preventDefault()});window.addEventListener("drop",e=>{if(e.dataTransfer.files.length){e.preventDefault();importFiles(e.dataTransfer.files)}});
window.addEventListener("beforeunload",e=>{if(dirty){e.preventDefault();e.returnValue=""}});
switchMode("library");
})();
