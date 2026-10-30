from pathlib import Path
import math,json
r=Path(__file__).resolve().parent
base=[(1,0),(8,1),(4,1),(2,1),(1,1),(1,2),(1,4),(1,8)]
directions=[]
for quarter in range(4):
 for x,y in base:
  for _ in range(quarter):x,y=-y,x
  directions.append((x,y))
angles=[math.atan2(y,x)%(2*math.pi) for x,y in directions];assert all(a<b for a,b in zip(angles,angles[1:]));primitives=[]
for i,((dx,dy),yaw) in enumerate(zip(directions,angles)):
 length=math.hypot(dx,dy)*.05;count=math.ceil(length/.0125)
 items=[(i,False,length,[[dx*.05*k/count,dy*.05*k/count,yaw] for k in range(1,count+1)])]
 for sign in [-1,1]:
  end=(i+sign)%32;delta=math.remainder(angles[end]-yaw,2*math.pi);assert delta*sign>0.;count=math.ceil(abs(delta)/.02)
  items.append((end,sign>0,0.,[[0.,0.,(yaw+delta*k/count)%(2*math.pi)] for k in range(1,count+1)]))
 for end,left,length,poses in items:
  primitives.append(dict(trajectory_id=len(primitives),start_angle_index=i,end_angle_index=end,left_turn=left,trajectory_radius=0.,trajectory_length=length,arc_length=0.,straight_length=length,poses=poses))
metadata=dict(motion_model='diff',turning_radius=.05,grid_resolution=.05,stopping_threshold=5,num_of_headings=32,heading_angles=angles,number_of_trajectories=len(primitives))
for p in primitives:
 if p['trajectory_length']:
  for x,y,yaw in p['poses']:assert x*math.cos(yaw)+y*math.sin(yaw)>0. and abs(-x*math.sin(yaw)+y*math.cos(yaw))<1e-9
 else:assert all(x==0. and y==0. for x,y,_ in p['poses'])
(r/'biped-straight-pivot32.json').write_text(json.dumps(dict(lattice_metadata=metadata,primitives=primitives),indent=2));print('32 true forward ray headings, 64 pure pivots; no lateral or reverse primitives')
