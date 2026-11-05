from pathlib import Path
import json,cv2,numpy as np,datetime,os,shutil
root=Path(__file__).resolve().parent
sources={'cam1':'20261030_211833_cam1','cam2':'20261030_211837'}
results={};images=[]
for camera,folder in sources.items():
 p=root/'measurements'/folder/'measurement.json';d=json.loads(p.read_text());assert len(d['samples'])>=30
 poses=[]
 for s in d['samples']:
  R=cv2.Rodrigues(np.array(s['rvec']))[0];t=np.array(s['translation_m']);T=np.eye(4);T[:3,:3]=R.T;T[:3,3]=np.array([.946,.154,0])-R.T@t;poses.append(T)
 poses=np.array(poses);U,_,V=np.linalg.svd(poses[:,:3,:3].mean(0));R=U@np.diag([1,1,np.linalg.det(U@V)])@V;T=np.eye(4);T[:3,:3]=R;T[:3,3]=np.median(poses[:,:3,3],axis=0)
 result={'parent_frame':'table','child_frame':d['samples'][0]['frame_id'],'transform_convention':f'p_table = T_table_{camera}_color_optical @ p_{camera}_color_optical',f'T_table_{camera}_color_optical':T.tolist(),'marker':{'dictionary':'DICT_4X4_50','id':10,'edge_m':.07,'center_table_m':[.946,.154,0.],'printed_top_direction':'+Y','normal_direction':'+Z'},'table_frame':{'origin':'lower-left tabletop corner viewed from above','x':'right','y':'up in top view','z':'up vertically','table_size_m':[1.8,.75]},'assumptions':['Marker remains at previously supplied table position and canonical orientation','Marker surface assumed at table Z=0'],'source':str(p),'samples':len(poses),'camera_position_sample_max_deviation_mm':float(max(np.linalg.norm(poses[:,:3,3]-T[:3,3],axis=1)*1000)),'median_reprojection_rms_px':float(np.median([s['reprojection_rms_px'] for s in d['samples']])),'independently_validated':False,'validated_for_robot_execution':False,'robot_transform_available':False}
 results[camera]=result
 im=cv2.imread(str(p.parent/(camera+'_marker.png')));images.append(im)
 print(camera,'table position cm:',(T[:3,3]*100).round(2).tolist(),'max sample deviation mm:',round(result['camera_position_sample_max_deviation_mm'],2))
# Save complete version first, then publish the active manifest atomically.
session=root/'saved'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S');session.mkdir(parents=True)
for c,r in results.items():
 dst=session/(c+'_table_calibration.json');dst.write_text(json.dumps(r,ensure_ascii=False,indent=2,allow_nan=False));shutil.copy2(dst,root/dst.name)
manifest={'calibrations':{c:str(session/(c+'_table_calibration.json')) for c in results},'independently_validated':False}
tmp=root/'active.tmp';tmp.write_text(json.dumps(manifest,ensure_ascii=False,indent=2));os.replace(tmp,root/'active.json')
cv2.imwrite(str(session/'measurement_review.jpg'),np.hstack(images))
print('Saved:',session)
