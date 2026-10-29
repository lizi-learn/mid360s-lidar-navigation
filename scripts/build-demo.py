#!/usr/bin/env python3
"""Build a static synthetic preview; no real scans or device identifiers."""
import pathlib,json,sys
root=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(root))
from demo_server import make_scene
_,state=make_scene()
source=(root/'web/index.html').read_text()
shim='''
const syntheticState=STATE;
const freeDemoCells=new Set(syntheticState.free_cells.map(([r,c])=>r*syntheticState.grid.size+c));
async function demoRequest(url,options={}){
 const result=(data,ok=true)=>({ok,json:async()=>data});
 if(url==='/api/status')return result(syntheticState);
 if(url!=='/api/goal')return result({error:'合成演示不进行真实录包或硬件操作'},false);
 const goal=JSON.parse(options.body),g=syntheticState.grid,n=g.size;
 const row=Math.floor((goal.y-g.origin_xy[1])/g.resolution),col=Math.floor((goal.x-g.origin_xy[0])/g.resolution);
 const sr=Math.floor((syntheticState.sensor_xy[1]-g.origin_xy[1])/g.resolution),sc=Math.floor((syntheticState.sensor_xy[0]-g.origin_xy[0])/g.resolution),begin=sr*n+sc,end=row*n+col;
 if(!Number.isFinite(goal.x)||!Number.isFinite(goal.y)||row<0||row>=n||col<0||col>=n||!freeDemoCells.has(end))return result({error:'目标位于障碍或未观察地面'},false);
 const parent=new Int32Array(n*n).fill(-1),queue=[begin];parent[begin]=begin;
 for(let at=0;at<queue.length&&parent[end]===-1;at++){
  const cell=queue[at],r=Math.floor(cell/n),c=cell%n;
  for(const[dr,dc]of [[1,0],[-1,0],[0,1],[0,-1]]){
   const rr=r+dr,cc=c+dc,next=rr*n+cc;
   if(rr>=0&&rr<n&&cc>=0&&cc<n&&parent[next]===-1&&freeDemoCells.has(next)){parent[next]=cell;queue.push(next)}
  }
 }
 if(parent[end]===-1)return result({error:'合成地面内没有连通路径'},false);
 const path=[];for(let cur=end;;cur=parent[cur]){path.push([g.origin_xy[0]+(cur%n+.5)*g.resolution,g.origin_xy[1]+(Math.floor(cur/n)+.5)*g.resolution]);if(cur===begin)break}path.reverse();
 syntheticState.path=path;syntheticState.goal=[goal.x,goal.y];syntheticState.path_state='合成网格路径有效';
 return result({path,goal:syntheticState.goal,start_gap_m:0,execution_enabled:false,demo:true});
}
'''.replace('STATE',json.dumps(state,ensure_ascii=False,separators=(',',':')))
source=source.replace('<script>','<script>'+shim,1).replace('await fetch(','await demoRequest(')
(root/'docs/demo.html').write_text(source);(root/'docs/.nojekyll').touch()
print('Built synthetic docs/demo.html')
