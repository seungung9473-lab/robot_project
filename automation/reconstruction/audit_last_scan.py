"""Rebuild aligned raw points and audit shape retention; read-only robot operation."""
import argparse, json
from pathlib import Path
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from placement_geometry import depth_table,object_region
from inclined_geometry import planes
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('session',type=Path);a=p.parse_args();s=a.session.resolve();out=s/'shape_audit';out.mkdir(exist_ok=True);ctx=json.loads((s/'spatial_context.json').read_text());axis=np.array(ctx['axis_point_table_m']);top=ctx['top_height_m'];manifest=json.loads((s/'session.json').read_text());merged=[];metadata=[];reports=[]
for oi,obj in enumerate(sorted((s/'objects').glob('object_*')),1):
 parts=[];rows=[]
 for vi,v in enumerate(manifest['views'],1):
  folder=s/v['path'];md=json.loads((folder/'metadata.json').read_text());R=Rotation.from_euler('z',-md['turntable']['angle_deg'],degrees=True).as_matrix()
  for ci,cam in enumerate(['cam1','cam2']):
   d=md['cameras'][cam];raw=cv2.imread(str(folder/d['depth_file']),-1);rgb=cv2.cvtColor(cv2.imread(str(folder/d['color_file'])),cv2.COLOR_BGR2RGB);P,valid=depth_table(raw,d['depth_scale_m'],d['camera_info'],ctx['calibrations'][cam][f'T_table_{cam}_color_optical']);mask=cv2.imread(str(obj/'sam2'/v['path']/(cam+'_mask.png')),0)>127;keep=valid&mask;points=P[keep];colors=rgb[keep];roi=object_region(points,ctx);points=(points-axis)@R.T+axis
   part={'points':points,'colors_rgb':colors,'roi':roi,'object_id':np.full(len(points),oi,np.uint8),'view_index':np.full(len(points),vi,np.uint8),'camera_id':np.full(len(points),ci,np.uint8)};parts.append(part);merged.append(part);rows.append({'view':v['path'],'camera':cam,'raw_points':len(points),'roi_points':int(roi.sum()),'measured_angle_deg':md['turntable']['angle_deg']})
 keys=parts[0].keys();data={k:np.concatenate([v[k] for v in parts]) for k in keys};np.savez_compressed(out/(obj.name+'_raw_aligned.npz'),**data);P=data['points'][data['roi']];Z=(P[:,2]-top)*1000;fused=np.load(obj/'analysis/cloud.npz')['points'];fz=(fused[:,2]-top)*1000
 # Upper envelope: p90 in1.5mm XY cells, >=10 samples, at least3mm inside XY hull.
 cell=.0015;origin=P[:,:2].min(0);key=np.floor((P[:,:2]-origin)/cell).astype(int);unique,inv=np.unique(key,axis=0,return_inverse=True);env=[]
 for i,k in enumerate(unique):
  z=P[inv==i,2]
  if len(z)>=10:env.append([*(origin+(k+.5)*cell),np.percentile(z,90),len(z)])
 env=np.array(env);hull=ConvexHull(P[:,:2]);clear=-(env[:,:2]@hull.equations[:,:2].T+hull.equations[:,2]).max(1);E=env[clear>=.003];E=E[(E[:,2]>=np.percentile(P[:,2],65))]
 # Compare a flat top, a single ramp, and a gable (two opposite slopes).
 xy=E[:,:2]-np.median(E[:,:2],0);zz=E[:,2];flat=float(np.sqrt(np.mean((zz-zz.mean())**2)));coef=np.linalg.lstsq(np.c_[xy,np.ones(len(xy))],zz,rcond=None)[0];planerms=float(np.sqrt(np.mean((np.c_[xy,np.ones(len(xy))]@coef-zz)**2)));best=None
 for angle in np.arange(0,180,2):
  u=np.array([np.cos(np.deg2rad(angle)),np.sin(np.deg2rad(angle))]);v=np.array([-u[1],u[0]]);x=xy@u;y=xy@v
  for b in np.linspace(np.percentile(x,25),np.percentile(x,75),21):
   A=np.c_[np.ones(len(x)),x,y,-np.abs(x-b)];c=np.linalg.lstsq(A,zz,rcond=None)[0]
   if c[3]<=0 or not c[1]+c[3]>0 or not c[1]-c[3]<0:continue
   rms=float(np.sqrt(np.mean((A@c-zz)**2)))
   if best is None or rms<best['rms_m']:best={'rms_m':rms,'cross_ridge_angle_deg':int(angle),'left_slope_deg':float(np.degrees(np.arctan(c[1]+c[3]))),'right_slope_deg':float(np.degrees(np.arctan(c[1]-c[3]))),'ridge_offset_m':float(b),'coefficients':c.tolist(),'profile_x':x,'profile_z':zz,'fitted':A@c}
 raw_planes=[]
 # Use first raw camera view, independently of voxel averaging and postfilters.
 for cam_id in [0,1]:
  rawP=data['points'][(data['view_index']==1)&(data['camera_id']==cam_id)&data['roi']]
  raw_planes.append({'camera':'cam'+str(cam_id+1),'planes':[{'tilt_from_horizontal_deg':float(np.degrees(np.arccos(np.clip(n[2],-1,1)))),'inliers':len(pp),'rms_mm':float(np.sqrt(np.mean((pp@n+d)**2))*1000)} for pp,n,d in planes(rawP,max_planes=6)]})
 q=[0,1,5,25,50,75,90,95,99,100];report={'object':obj.name,'raw_depth_valid_points':len(data['points']),'roi_points':len(P),'stored_fused_points':len(fused),'raw_roi_height_percentiles_mm':dict(zip(map(str,q),np.percentile(Z,q).tolist())),'stored_height_percentiles_mm':dict(zip(map(str,q),np.percentile(fz,q).tolist())),'upper_region_definition':'height>=raw ROI Z P75','upper_region_height_percentiles_mm':dict(zip(['min','p25','p50','p75','max'],np.percentile(Z[Z>=np.percentile(Z,75)],[0,25,50,75,100]).tolist())),'upper_envelope_cells':len(E),'upper_envelope_flat_rms_mm':flat*1000,'upper_envelope_single_plane_rms_mm':planerms*1000,'gable_fit':({k:v for k,v in best.items() if k not in ['profile_x','profile_z','fitted']} if best else None),'raw_first_view_planes':raw_planes,'per_view_counts':rows}
 if best:
  report['gable_fit']['rms_mm']=best['rms_m']*1000;report['gable_fit']['improvement_over_flat_percent']=(1-best['rms_m']/flat)*100
 # A fit is diagnostic only; do not declare a gable from a tiny noisy improvement.
 report['roof_assessment']='gable retained' if best and best['rms_m']<flat*.7 and best['left_slope_deg']>15 and best['right_slope_deg']<-15 else 'gable not established by merged upper envelope; inspect raw planes and multi-view alignment'
 reports.append(report)
 fig,axs=plt.subplots(1,3,figsize=(15,4.5));C=data['colors_rgb'][data['roi']]/255;center=np.median(P[:,:2],0)
 for ax,(u,v),title in zip(axs,[(0,1),(0,2),(1,2)],['Top XY','Side XZ','Side YZ']):
  Q=P.copy();Q[:,:2]-=center;Q[:,2]-=top;ax.scatter(Q[:,u]*1000,Q[:,v]*1000,c=C,s=.2,rasterized=True);ax.set_aspect('equal');ax.set_title(obj.name+' raw aligned / '+title);ax.set_xlabel('XYZ'[u]+' mm');ax.set_ylabel(('height above turntable' if v==2 else 'Y')+' mm');ax.grid(alpha=.2)
 fig.tight_layout();fig.savefig(out/(obj.name+'_raw_views.png'),dpi=180);plt.close(fig)
 fig,axs=plt.subplots(1,3,figsize=(14,4));axs[0].hist(Z,bins=np.arange(0,max(Z)+1,1));axs[0].set_xlabel('Height above turntable (mm)');axs[0].set_ylabel('Raw point count');axs[1].scatter((E[:,0]-center[0])*1000,(E[:,1]-center[1])*1000,c=(E[:,2]-top)*1000,s=12);axs[1].set_aspect('equal');axs[1].set_title('Upper envelope / height colour')
 if best:axs[2].scatter(best['profile_x']*1000,(best['profile_z']-top)*1000,s=8,label='upper-envelope cells');idx=np.argsort(best['profile_x']);axs[2].plot(best['profile_x'][idx]*1000,(best['fitted'][idx]-top)*1000,'.',ms=2,label='gable fit');axs[2].legend(fontsize=7)
 axs[2].set_xlabel('Cross-ridge (mm)');axs[2].set_ylabel('Height above turntable (mm)');fig.tight_layout();fig.savefig(out/(obj.name+'_height_analysis.png'),dpi=160);plt.close(fig)
all_data={k:np.concatenate([p[k] for p in merged]) for k in merged[0]};np.savez_compressed(out/'all_objects_raw_aligned.npz',**all_data);roi=all_data['roi'];np.savez_compressed(out/'all_objects_roi_aligned_no_voxel.npz',**{k:v[roi] for k,v in all_data.items() if k!='roi'})
P=all_data['points'];Q=P.copy();Q[:,:2]-=axis[:2];Q[:,2]-=top;C=all_data['colors_rgb']/255;fig,axs=plt.subplots(1,3,figsize=(16,5))
for ax,(u,v),title in zip(axs,[(0,1),(0,2),(1,2)],['Top XY','Side XZ','Side YZ']):
 ax.scatter(Q[:,u]*1000,Q[:,v]*1000,c=C,s=.15,rasterized=True);ax.set_aspect('equal');ax.set_title(title+' / raw aligned, no voxel');ax.set_xlabel('XYZ'[u]+' mm');ax.set_ylabel('XYZ'[v]+' mm');ax.set_xlim(-110,110);ax.set_ylim((-110,110) if v==1 else (-10,100));ax.grid(alpha=.25)
fig.suptitle('Last scan: 12 angles x 2 cameras | zoom around turntable; outliers outside view retained in NPZ');fig.tight_layout();fig.savefig(out/'all_objects_raw_top_and_sides.png',dpi=180);plt.close(fig)
(out/'height_and_shape_report.json').write_text(json.dumps(reports,indent=2));(out/'raw_point_provenance.json').write_text(json.dumps({'session':str(s),'file':'all_objects_raw_aligned.npz','points':len(P),'roi_points':int(roi.sum()),'regenerated_from':'original RGBD images + saved SAM2 masks + frozen extrinsics + recorded encoder angles','not_original_persisted_merged_file':True,'voxel_applied':False,'outlier_or_component_filters_applied':False,'ROI_mask_stored_separately':True,'original_masks_already_include_any_saved_overlap_exclusions':True,'camera_depth_validity_m':[.08,.9]},indent=2));print(json.dumps([{k:v for k,v in r.items() if k not in ['raw_first_view_planes','per_view_counts']} for r in reports],indent=2))
