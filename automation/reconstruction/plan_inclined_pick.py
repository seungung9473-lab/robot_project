"""Reconstructed pair -> cross-camera inclined faces -> table alignment and IK. Offline."""
import argparse,json
from pathlib import Path
import numpy as np,yaml,cv2
from placement_geometry import depth_table
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from inclined_geometry import PLANE_INLIER_TOLERANCE_M,PATCH_RMS_LIMIT_MM,planes,patch,alignment_angles,rotate_surface,compensated_axis,solve_axis
from motion.kinematics import fk,m,W
lo=np.array([j["limit"]["lower"] for j in m["joints"]],dtype=float);hi=np.array([j["limit"]["upper"] for j in m["joints"]],dtype=float)
p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--object');a=p.parse_args();s=a.session.resolve();wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());axis=wc['turntable']['axis_point_table_m'];j1=(W@np.r_[[-.0177,.0177,.1598],1])[:3];results=[]
for obj in sorted((s/'objects').glob('object_*')):
    if a.object and obj.name!=a.object:continue
    # Fit individual stopped observations; fusion can thicken inclined surfaces.
    # Keep exact encoder-unrotated points for independent cross-camera checks.
    observations={c:[] for c in ['cam1','cam2']};candidates={c:[] for c in observations}
    manifest=json.loads((s/'session.json').read_text())
    for view in manifest['views']:
        folder=s/view['path'];md=json.loads((folder/'metadata.json').read_text());rot=Rotation.from_euler('z',-md['turntable']['angle_deg'],degrees=True).as_matrix()
        for cam in observations:
            meta=md['cameras'][cam];raw=cv2.imread(str(folder/meta['depth_file']),-1);P,valid=depth_table(raw,meta['depth_scale_m'],meta['camera_info'],wc['cameras'][cam]['T_table_optical']);mask=cv2.imread(str(obj/'sam2'/view['path']/(cam+'_mask.png')),0)>127
            pts=P[valid&mask];pts=(pts-np.asarray(axis))@rot.T+axis
            if len(pts)<120:continue
            key=view['path']+'/'+cam;observations[cam].append((key,pts))
            for pp,n,d in planes(pts,max_planes=5):
                record=patch(pts,n,d,prefer_high=True)
                if record is not None:
                    record['observation']=key;candidates[cam].append(record)
    clouds={c:np.vstack([pts for _,pts in values]) for c,values in observations.items()}
    pairs=[]
    for first in candidates['cam1']:
        if not first['geometry_pass']:continue
        p0=np.array(first['point_table_m']);n0=np.array(first['normal_table'])
        for second in candidates['cam2']:
            if not second['geometry_pass']:continue
            n1=np.array(second['normal_table']);angle=float(np.degrees(np.arccos(np.clip(n0@n1,-1,1))))
            if angle>5:continue
            first_points=dict(observations['cam1'])[first['observation']];other_points=dict(observations['cam2'])[second['observation']]
            # Independent highest-point choices can miss the common interior of
            # a small lid. Test shared centres while retaining BOTH full-disk
            # coverage, 3mm edge margin, RMS, normal and gap checks.
            original=np.array(first['point_table_m']);other=np.array(second['point_table_m']);selected_first=None
            for proposed in [original,(original+other)/2,other]:
                candidate=proposed-n0*(proposed@n0+first['plane_d'])
                gap=abs(float(candidate@n1+second['plane_d']));near=float(cKDTree(other_points).query(candidate)[0])
                if gap>.003 or near>.003:continue
                one=patch(first_points,n0,first['plane_d'],center_point=candidate)
                two=patch(other_points,n1,second['plane_d'],center_point=candidate)
                if one is not None and two is not None and one['geometry_pass'] and two['geometry_pass']:
                    selected_first=dict(one,observation=first['observation']);p0=candidate;break
            if selected_first is None:continue
            if any(np.linalg.norm(np.asarray(old['face']['normal_table'])-n0)<.035 and np.linalg.norm(np.asarray(old['face']['point_table_m'])-p0)<.0015 for old in pairs):continue
            normal=(n0+n1)/np.linalg.norm(n0+n1);out={'face':selected_first,'other_camera_face':second,'cross_camera_normal_deg':angle,'cross_camera_plane_gap_mm':gap*1000,'cross_camera_point_distance_mm':near*1000,'alignment_options':[]}
            for turn in alignment_angles(p0,normal,axis,j1):
                point,n=rotate_surface(p0,normal,axis,turn);desired=compensated_axis(point,n,j1);contact=solve_axis(fk,lo,hi,point,desired)
                if contact is None:continue
                approach=solve_axis(fk,lo,hi,point+n*.02,desired,seed=contact[0])
                if approach is None:continue
                q=contact[0];qa=approach[0]
                out['alignment_options'].append({'turntable_angle_deg':turn,'point_table_m':point.tolist(),'normal_table':n.tolist(),'commanded_tool_axis_table':desired.tolist(),'contact_q_rad':q.tolist(),'approach_q_rad':qa.tolist(),'contact_position_error_mm':contact[1]*1000,'contact_axis_error_deg':contact[2],'approach_position_error_mm':approach[1]*1000,'approach_axis_error_deg':approach[2],'approach_clearance_m':.02,'lift_clearance_m':.03,'collision_checked':False})
            pairs.append(out)
    result={'object':obj.name,'session':str(obj),'per_camera_faces':candidates,'matched_faces':pairs};results.append(result);print(obj.name,'faces', {c:len(v) for c,v in candidates.items()},'matched',len(pairs),'reachable',sum(len(v['alignment_options']) for v in pairs),flush=True)
if a.object and (s/'inclined_pick_plan.json').exists():
    prior=json.loads((s/'inclined_pick_plan.json').read_text());results=[o for o in prior['objects'] if o['object']!=a.object]+results;results.sort(key=lambda o:o['object'])
report={'status':'geometry and IK only; collision planning required; contact remains precomputed after rotation','contact_preference':'upper feasible face region with full cup support, not low-edge maximum-clearance alone','max_face_tilt_deg':90,'cup_diameter_mm':25,'edge_margin_mm':3,'plane_inlier_tolerance_mm':PLANE_INLIER_TOLERANCE_M*1000,'patch_rms_limit_mm':PATCH_RMS_LIMIT_MM,'sag_compensation_deg':3,'objects':results,'robot_commands_sent':False};(s/'inclined_pick_plan.json').write_text(json.dumps(report,indent=2));print(s/'inclined_pick_plan.json',flush=True)
