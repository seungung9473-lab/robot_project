from pathlib import Path
import json,struct,numpy as np,cv2
r=Path(__file__).resolve().parents[1];src=Path('/home/seungung9473/robot_ws/src/soarm4_description');m=json.loads((src/'config/model.json').read_text());pose=json.loads((r/'robot_registration/robot_pose_candidate.json').read_text());pose['status']='Placement confirmed by user; visual preview only; no runtime transform published';pose['pending']=None;(r/'robot_registration/robot_pose_confirmed_layout.json').write_text(json.dumps(pose,ensure_ascii=False,indent=2));world=np.array(pose['candidate_T_table_urdf_world'])
def transl(p):
 T=np.eye(4);T[:3,3]=p;return T
def rot(axis,q):
 T=np.eye(4);T[:3,:3]=cv2.Rodrigues(np.array(axis,dtype=float)*q)[0];return T
links={'electronics_base_link':np.eye(4),'base_link':transl(m['robot_base_xyz'])@rot([0,0,1],m['robot_base_yaw'])}
pivots=[]
for j,q in zip(m['joints'],m['poses']['home']):
 T=links[j['parent']]@transl(j['origin_xyz']);pivots.append((world@T)[:3,3].round(5).tolist());links[j['child']]=T@rot(j['axis'],q)
meshes=[]
for v in m['visuals']:
 data=(src/'meshes'/v['file']).read_bytes();n=struct.unpack_from('<I',data,80)[0];dt=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')]);tri=np.frombuffer(data,dt,n,84)['v'];flat=tri.reshape(-1,3).astype(float)
 # Preview-only vertex clustering; original STL unchanged.
 key=np.round(flat/.0035).astype(int);_,inv=np.unique(key,axis=0,return_inverse=True);N=inv.max()+1;verts=np.column_stack([np.bincount(inv,weights=flat[:,k],minlength=N)/np.bincount(inv,minlength=N) for k in range(3)]);faces=inv.reshape(-1,3);faces=faces[(faces[:,0]!=faces[:,1])&(faces[:,1]!=faces[:,2])&(faces[:,0]!=faces[:,2])];faces=np.unique(np.sort(faces,axis=1),axis=0);T=world@links[v['link']];verts=verts@T[:3,:3].T+T[:3,3];meshes.append({'v':verts.round(4).tolist(),'f':faces.tolist(),'part':v['name'],'tone':'dark' if 'servo' in v['name'] else 'robot','id':'robot'})
active=json.loads((r/'active.json').read_text());cams={}
for c,p in active['calibrations'].items():
 d=json.loads(Path(p).read_text());cams[c]={'T':d[f'T_table_{c}_color_optical'],'K':d['camera_info']['K'],'size':[d['camera_info']['width'],d['camera_info']['height']],'rms':d['median_reprojection_rms_px']}
padact=json.loads((r/'placement_pads/active.json').read_text());pads={}
for c,p in padact['pads'].items():
 d=json.loads(Path(p).read_text());pads[c]={'pos':d['placement_center_table_m'],'id':d['marker_id']}
layout=json.loads((r/'marker_layout.json').read_text());markers=layout['fixed_markers'];dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50);bits={str(i):(cv2.aruco.drawMarker(dictionary,i,60).reshape(6,10,6,10).mean((1,3))>127).astype(int).tolist() for i in [7,8,10,11]}
tcp=(world@links['tool_link']@transl(m['tcp_xyz']))[:3,3].tolist()
d={'meshes':meshes,'cameras':cams,'pads':pads,'markers':markers,'bits':bits,'turntable':layout['turntable_mechanical_reference'],'robot':{'j1':pose['candidate_joint1_pivot_table_m'],'base':pose['candidate_base_link_origin_table_m'],'corner':pose['table_enclosure_bottom_lower_left_m'],'pivots':pivots,'tcp':tcp,'home':m['poses']['home']}}
p=Path(__file__).parent/'scene.json';p.write_text(json.dumps(d,ensure_ascii=False,separators=(',',':')));print('scene bytes',p.stat().st_size,'triangles',sum(len(x['f']) for x in meshes),'TCP',tcp)
