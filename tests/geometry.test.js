const assert=require("node:assert/strict"), fs=require("node:fs"), path=require("node:path"), vm=require("node:vm");
const root=path.resolve(__dirname,"..");
const context=vm.createContext({});
vm.runInContext(fs.readFileSync(path.join(root,"visualization/geometry.js"),"utf8")+"\nthis.G=Geo;",context);
const G=context.G;
for(let w=1;w<=9;w++)for(let h=1;h<=8;h++){
 const g=Array.from({length:h},(_,y)=>Array.from({length:w},(_,x)=>(x*13+y*7+x*y)%11===3?"#":".").join(""));
 const c=G.clearance(g);
 for(let y=0;y<h;y++)for(let x=0;x<w;x++){
  let k=0;while(Array.from({length:2*k+1},(_,dy)=>Array.from({length:2*k+1},(_,dx)=>g[y+dy-k]?.[x+dx-k]===".").every(Boolean)).every(Boolean))k++;
  assert.equal(c[y*w+x],Math.max(0,2*k-1));
 }
}
assert.deepEqual(Array.from(G.flood([".#."],G.clearance([".#."]),1).counts),[1,1]);
assert.deepEqual(Array.from(G.flood(["..."],G.clearance(["..."]),1,0).labels),[0,1,2]);
assert.deepEqual(Array.from(G.flood([".#."],G.clearance([".#."]),1,0).labels),[0,-1,-1]);
assert.equal(G.parseGrid("type octile\nheight 1\nwidth 4\nmap\n.G@T")[0],"..##");
assert.throws(()=>G.parseGrid("..\n."));
assert.throws(()=>G.validate(["...","...","..."],[{id:0,size:3,start:[0,0],goal:null}]));
assert.throws(()=>G.validate(["...","...","..."],[{id:0,size:3,start:[1,1],goal:null},{id:1,size:1,start:[2,2],goal:null}]));
const g=G.parseGrid(fs.readFileSync(path.join(root,"maps/benchmarks/random-128-128-10-la.map"),"utf8"));
assert.equal(Array.from(G.clearance(g)).filter(s=>s>=5).length,6276);
console.log("Geometry tests passed");
