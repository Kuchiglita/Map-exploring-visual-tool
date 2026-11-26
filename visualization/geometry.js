const Geo = (function geometryCore(){
"use strict";
const fail=m=>{throw new Error(m)};
function parseGrid(text){
 let lines=text.replace(/^\uFEFF/,"").trim().split(/\r?\n/).map(s=>s.trim());
 if(/^type\s/i.test(lines[0])){
  const h=/^height\s+(\d+)$/i.exec(lines[1]||""), w=/^width\s+(\d+)$/i.exec(lines[2]||"");
  if(!h||!w||lines[3]?.toLowerCase()!=="map")fail("Некорректный заголовок MovingAI");
  lines=lines.slice(4); if(lines.length!==+h[1]||lines.some(s=>s.length!==+w[1]))fail("Размеры не совпадают с заголовком");
  lines=lines.map(s=>[...s].map(c=>".GS".includes(c)?".":"@OTW".includes(c)?"#":fail("Неизвестный символ MovingAI: "+c)).join(""));
 }
 if(!lines.length||!lines[0].length||lines.some(s=>s.length!==lines[0].length||!/^[.#]+$/.test(s)))fail("Нужна прямоугольная матрица из . и #");
 if(lines.length*lines[0].length>1000000)fail("Лимит: 1 000 000 клеток");
 return lines;
}
const dims=g=>[g[0].length,g.length];
function clearance(g){
 const [w,h]=dims(g),pw=w+2,ph=h+2,d=new Int32Array(pw*ph).fill(-1),q=new Int32Array(d.length);
 let tail=0;for(let y=0;y<ph;y++)for(let x=0;x<pw;x++)if(!x||!y||x===pw-1||y===ph-1||g[y-1][x-1]==="#"){let i=y*pw+x;d[i]=0;q[tail++]=i}
 for(let head=0;head<tail;head++){let i=q[head],x=i%pw,y=(i/pw)|0;for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){let xx=x+dx,yy=y+dy,j=yy*pw+xx;if(xx>=0&&xx<pw&&yy>=0&&yy<ph&&d[j]===-1){d[j]=d[i]+1;q[tail++]=j}}}
 return Int32Array.from({length:w*h},(_,i)=>{let k=d[(((i/w)|0)+1)*pw+i%w+1];return k?2*k-1:0});
}
function flood(g,c,size,start=null){
 const [w,h]=dims(g), labels=new Int32Array(w*h).fill(-1),q=new Int32Array(w*h),counts=[];
 const visit=(s,label)=>{let tail=1;q[0]=s;labels[s]=label;for(let head=0;head<tail;head++){let i=q[head],x=i%w,y=(i/w)|0;for(const j of [x?i-1:-1,x<w-1?i+1:-1,y?i-w:-1,y<h-1?i+w:-1])if(j>=0&&c[j]>=size&&labels[j]===-1){labels[j]=start===null?label:labels[i]+1;q[tail++]=j}}return tail};
 if(start!==null){if(start>=0&&start<c.length&&c[start]>=size)visit(start,0);return {labels,counts}}
 for(let i=0;i<c.length;i++)if(c[i]>=size&&labels[i]===-1)counts.push(visit(i,counts.length));
 return {labels,counts};
}
const validSize=s=>Number.isInteger(s)&&s>=1&&s%2===1;
function fits(g,c,p,s){const [w,h]=dims(g);return validSize(s)&&Array.isArray(p)&&p.length===2&&p.every(Number.isInteger)&&p[0]>=0&&p[0]<w&&p[1]>=0&&p[1]<h&&c[p[1]*w+p[0]]>=s}
function overlaps(p,s,q,t){return Math.abs(p[0]-q[0])<(s+t)/2&&Math.abs(p[1]-q[1])<(s+t)/2}
function validate(g,agents){
 const c=clearance(g),ids=new Set();if(!Array.isArray(agents))fail("agents должен быть массивом");
 for(const a of agents){if(!a||!Number.isSafeInteger(a.id)||a.id<0||ids.has(a.id)||!validSize(a.size)||!fits(g,c,a.start,a.size)||a.goal!==null&&!fits(g,c,a.goal,a.size))fail("Некорректный агент: "+a?.id);ids.add(a.id)}
 for(let i=0;i<agents.length;i++)for(let j=0;j<i;j++)if(overlaps(agents[i].start,agents[i].size,agents[j].start,agents[j].size))fail("Начальные позиции агентов пересекаются");
 return c;
}
return {parseGrid,dims,clearance,flood,validSize,fits,overlaps,validate};
})();
