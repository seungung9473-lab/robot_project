"""Reconstructed pair -> cross-camera inclined faces -> table alignment and IK. Offline."""
import argparse,json
from pathlib import Path
import numpy as np,yaml
from scipy.spatial import cKDTree
from inclined_geometry import PLANE_INLIER_TOLERANCE_M,PATCH_RMS_LIMIT_MM,planes,patch,alignment_angles,rotate_surface,compensated_axis,solve_axis
from motion.kinematics import fk,m,W
from contact_offset import CONTACT_RIGHT_M,shifted_point
lo=np.array([j["limit"]["lower"] for j in m["joints"]],dtype=float);hi=np.array([j["limit"]["upper"] for j in m["joints"]],dtype=float)
p=argparse.ArgumentParser();p.add_argument('session',type=Path);p.add_argument('--object');a=p.parse_args();s=a.session.resolve();wc=yaml.safe_load((s/'workcell_snapshot.yaml').read_text());axis=wc['turntable']['axis_point_table_m'];j1=(W@np.r_[[-.0177,.0177,.1598],1])[:3];results=[]
from shared_observations import collect_plane_observations
objects=[obj for obj in sorted((s/'objects').glob('object_*')) if not a.object or obj.name==a.object]
all_observations=collect_plane_observations(s,objects,wc)
for obj in objects:
    observations=all_observations.pop(obj.name);candidates={c:[] for c in observations}
    for cam,entries in observations.items():
        for key,pts in entries:
            for pp,n,d in planes(pts,max_planes=5):
                record=patch(pts,n,d,prefer_high=True)
                if record is not None:
                    record['observation']=key;candidates[cam].append(record)
    lookup={cam:dict(entries) for cam,entries in observations.items()};other_trees={}
    pairs=[]
    for first in candidates['cam1']:
        if not first['geometry_pass']:continue
        p0=np.array(first['point_table_m']);n0=np.array(first['normal_table'])
        for second in candidates['cam2']:
            if not second['geometry_pass']:continue
            n1=np.array(second['normal_table']);angle=float(np.degrees(np.arccos(np.clip(n0@n1,-1,1))))
            if angle>5:continue
            first_points=lookup['cam1'][first['observation']];other_points=lookup['cam2'][second['observation']]
            if second['observation'] not in other_trees:other_trees[second['observation']]=cKDTree(other_points)
            other_tree=other_trees[second['observation']]
            # Independent highest-point choices can miss the common interior of
            # a small lid. Test shared centres while retaining BOTH full-disk
            # coverage, 3mm edge margin, RMS, normal and gap checks.
            original=np.array(first['point_table_m']);other=np.array(second['point_table_m']);selected_first=None
            for proposed in [original,(original+other)/2,other]:
                candidate=proposed-n0*(proposed@n0+first['plane_d'])
                gap=abs(float(candidate@n1+second['plane_d']));near=float(other_tree.query(candidate)[0])
                if gap>.003 or near>.003:continue
                one=patch(first_points,n0,first['plane_d'],center_point=candidate)
                two=patch(other_points,n1,second['plane_d'],center_point=candidate)
                if one is not None and two is not None and one['geometry_pass'] and two['geometry_pass']:
                    selected_first=dict(one,observation=first['observation']);p0=candidate;break
            if selected_first is None:continue
            if any(np.linalg.norm(np.asarray(old['face']['normal_table'])-n0)<.035 and np.linalg.norm(np.asarray(old['face']['point_table_m'])-p0)<.0015 for old in pairs):continue
            normal=(n0+n1)/np.linalg.norm(n0+n1);out={'face':selected_first,'other_camera_face':second,'cross_camera_normal_deg':angle,'cross_camera_plane_gap_mm':gap*1000,'cross_camera_point_distance_mm':near*1000,'alignment_options':[]}
            for turn in alignment_angles(p0,normal,axis,j1):
                point,n=rotate_surface(p0,normal,axis,turn);offset_note='skipped_cup_would_leave_face'
                # 모든 물체를 중심보다 살짝 왼쪽으로 잡는 편향의 소프트웨어 보정: 팔 기준 오른쪽으로 최대 CONTACT_RIGHT_M.
                # 컵이 그 면에 들어가는 가장 큰 이동량(5→1 mm)만 적용하고, 하나도 안 되면 원래 점을 쓴다.
                for step in range(5,0,-1):
                    right_m=CONTACT_RIGHT_M*step/5
                    point_s=shifted_point(point,n,j1,right_m);p0_s=rotate_surface(point_s,n,axis,-turn)[0]
                    p0_s=p0_s-n0*(p0_s@n0+first['plane_d'])
                    fit_one=patch(lookup['cam1'][first['observation']],n0,first['plane_d'],center_point=p0_s);fit_two=patch(lookup['cam2'][second['observation']],n1,second['plane_d'],center_point=p0_s)
                    if fit_one is not None and fit_two is not None and fit_one['geometry_pass'] and fit_two['geometry_pass']:
                        point=point_s;offset_note='right_%.1fmm'%(right_m*1000);break
                desired=compensated_axis(point,n,j1);contact=solve_axis(fk,lo,hi,point,desired)
                if contact is None:continue
                approach=solve_axis(fk,lo,hi,point+n*.02,desired,seed=contact[0])
                if approach is None:continue
                q=contact[0];qa=approach[0]
                out['alignment_options'].append({'contact_lateral_offset':offset_note,'turntable_angle_deg':turn,'point_table_m':point.tolist(),'normal_table':n.tolist(),'commanded_tool_axis_table':desired.tolist(),'contact_q_rad':q.tolist(),'approach_q_rad':qa.tolist(),'contact_position_error_mm':contact[1]*1000,'contact_axis_error_deg':contact[2],'approach_position_error_mm':approach[1]*1000,'approach_axis_error_deg':approach[2],'approach_clearance_m':.02,'lift_clearance_m':.03,'collision_checked':False})
            pairs.append(out)
    result={'object':obj.name,'session':str(obj),'per_camera_faces':candidates,'matched_faces':pairs};results.append(result);print(obj.name,'faces', {c:len(v) for c,v in candidates.items()},'matched',len(pairs),'reachable',sum(len(v['alignment_options']) for v in pairs),flush=True)
if a.object and (s/'inclined_pick_plan.json').exists():
    prior=json.loads((s/'inclined_pick_plan.json').read_text());results=[o for o in prior['objects'] if o['object']!=a.object]+results;results.sort(key=lambda o:o['object'])
report={'status':'geometry and IK only; collision planning required; contact remains precomputed after rotation','contact_preference':'upper feasible face region with full cup support, not low-edge maximum-clearance alone','max_face_tilt_deg':90,'cup_diameter_mm':25,'edge_margin_mm':3,'plane_inlier_tolerance_mm':PLANE_INLIER_TOLERANCE_M*1000,'patch_rms_limit_mm':PATCH_RMS_LIMIT_MM,'sag_compensation_deg':3,'objects':results,'robot_commands_sent':False};(s/'inclined_pick_plan.json').write_text(json.dumps(report,indent=2));print(s/'inclined_pick_plan.json',flush=True)
