"""Occupancy-aware pad reservations; pure geometry plus IK, never actuation."""
import json
import time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from pad_packing import footprint, separated
from pad_occupancy import blockers, free_offsets, free_offsets_refined, inflate_polygon, FOOTPRINT_MARGIN_M, read_occupancy, write_snapshot


class StaticPlacementRejected(RuntimeError):
    def __init__(self, message, *, retryable=True, reason='geometry', blocked_ids=()):
        super().__init__(message)
        self.retryable = retryable
        self.reason = reason
        self.blocked_ids = list(blocked_ids)


def within_pad(points, center, radius, margin=.002):
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('Finite nonempty Nx3 payload required')
    center = np.asarray(center, float)
    if center.shape != (3,) or not np.isfinite(center).all() or not np.isfinite(radius) or not 0 <= margin < radius:
        raise ValueError('Invalid pad geometry')
    return bool(np.max(np.linalg.norm(points[:,:2]-center[:2],axis=1)) <= radius-margin)


def placement_ik(fk, lo, hi, target, seed, local, dims, j1, pitch):
    """Warm-start the existing placement residual; same tolerances, no random 21-start loop."""
    from payload_placement import signed_pitch
    def residual(q):
        tool = fk(q); body = tool @ local
        bottom = body[:3,3].copy(); bottom[2] -= np.sum(np.abs(body[2,:3])*dims)/2
        return np.r_[(bottom-target)*10,(signed_pitch(tool,j1)-pitch)*.3]
    fit = least_squares(residual,np.clip(seed,lo,hi),bounds=(lo,hi),max_nfev=100)
    error = residual(fit.x)
    if np.linalg.norm(error[:3]) >= .002 or abs(error[3]) >= np.radians(.2)*.3:
        return None
    return fit.x


def jointly_choose(options, gap=.002):
    order = sorted(options,key=lambda key:len(options[key])); chosen={}
    def visit(index):
        if index==len(order):return dict(chosen)
        key=order[index]
        for candidate in options[key]:
            p=np.asarray(candidate['reserved_polygon_table_m'])
            if all(separated(p,np.asarray(other['reserved_polygon_table_m']),gap) for other in chosen.values()):
                chosen[key]=candidate
                result=visit(index+1)
                if result is not None:return result
                del chosen[key]
        return None
    return visit(0)


def reserve_pad(session, object_keys, pad_id, kinematics, *, occupancy=None, time_limit=40.):
    """Search full pad on a 2mm grid; retain every observed hull vertex and margins."""
    from payload_placement import centered_payload_origin, signed_pitch
    if pad_id not in (5,6) or not object_keys or len(set(object_keys))!=len(object_keys):
        raise ValueError('Invalid pad reservation request')
    session=Path(session); path=session/f'pad_{pad_id}_placement_plan.json'
    if path.exists():raise RuntimeError('Reservation already exists; do not overwrite reviewed plans')
    wc=yaml.safe_load((session/'workcell_snapshot.yaml').read_text())
    report=json.loads((session/'inclined_pick_plan.json').read_text())
    fk,model,world=kinematics.fk,kinematics.m,kinematics.W
    occupancy=read_occupancy(world) if occupancy is None else occupancy
    write_snapshot(session,occupancy)
    lo=np.array([j['limit']['lower'] for j in model['joints']],float)
    hi=np.array([j['limit']['upper'] for j in model['joints']],float)
    axis=np.asarray(wc['turntable']['axis_point_table_m'])
    cfg=wc['pads']['pad2' if pad_id==5 else 'pad1'];pad=np.asarray(cfg['center_table_m'],float)
    radius=float(cfg['diameter_m'])/2;target=pad.copy()
    target[:2]+=wc['place_correction_m'].get('pad_ID'+str(pad_id),[0.,0.])
    j1=(world@np.r_[[-.0177,.0177,.1598],1])[:3]
    n=int(np.ceil(radius/.002));ticks=np.arange(-n,n+1)*.002
    grid=np.array([(x,y) for x in ticks for y in ticks]);grid=grid[np.argsort(np.linalg.norm(grid,axis=1),kind='stable')]
    started=time.monotonic();next_notice=started;options={};reviews=[];timed_out=False
    def note(message):
        nonlocal next_notice
        now=time.monotonic()
        if now>=next_notice:
            print(f'계산중: 패드 {pad_id} 빈자리 탐색 · {message} · {now-started:.0f}초',flush=True);next_notice=now+5
    for key in object_keys:
        entry=next(row for row in report['objects'] if row['object']==key)
        cloud=np.load(Path(entry['session'])/'analysis/cloud.npz')['points']
        hull_cloud=cloud[ConvexHull(cloud).vertices]  # exact envelope, no downsampling/shrinking
        choices=[opt for face in entry['matched_faces'] for opt in face['alignment_options']]
        candidates=[]
        for index,option in enumerate(choices):
            note(f'{key} 집기 후보 {index+1}/{len(choices)}')
            if time.monotonic()-started>time_limit:timed_out=True;break
            rotation=Rotation.from_euler('z',option['turntable_angle_deg'],degrees=True).as_matrix()
            points=(cloud-axis)@rotation.T+axis;vertices=(hull_cloud-axis)@rotation.T+axis
            low,high=np.percentile(points,.5,axis=0),np.percentile(points,99.5,axis=0)
            low[2]=wc['turntable']['top_height_m'];low[:2]-=.002;high+=.002;dims=high-low
            origin=np.eye(4);origin[:3,3]=centered_payload_origin(points,(high+low)/2)
            contact=np.asarray(option['contact_q_rad']);local=np.linalg.inv(fk(contact))@origin;pitch=signed_pitch(fk(contact),j1)
            nominal=None
            # At most three deterministic seeds to get a placement orientation for the spatial prefilter.
            seeds=[contact,(lo+hi)/2,np.clip(contact+[0,.2,-.2,0],lo,hi)]
            for seed in seeds:
                nominal=placement_ik(fk,lo,hi,target,seed,local,dims,j1,pitch)
                if nominal is not None:break
            if nominal is None:
                reviews.append({'object':key,'option':index,'result':'nominal_ik_unavailable'});continue
            body=fk(nominal)@local
            nominal_points=(vertices-origin[:3,3])@body[:3,:3].T+body[:3,3]
            free,fine_used=free_offsets_refined(inflate_polygon(footprint(nominal_points)),grid,pad,radius,occupancy)
            reviews.append({'object':key,'option':index,'prefilter_free_offsets':len(free),'fine_grid_used':fine_used})
            # Try spatially distinct locations before adjacent 2mm variants.
            ordered=[]
            if len(free):
                distance=np.full(len(free),np.inf);pick=0
                for _ in range(min(64,len(free))):
                    ordered.append(free[pick]);distance=np.minimum(distance,np.linalg.norm(free-free[pick],axis=1))
                    distance[pick]=-np.inf;pick=int(np.argmax(distance))
            seed=nominal
            for offset in ordered:
                note(f'{key} 기존 물체 회피 · 유효 위치 {len(candidates)}개')
                if time.monotonic()-started>time_limit:timed_out=True;break
                q=placement_ik(fk,lo,hi,target+[*offset,0.],seed,local,dims,j1,pitch)
                if q is None:continue
                seed=q;actual=fk(q)@local
                if np.degrees(np.arccos(np.clip(actual[2,2],-1,1)))>1:continue
                placed=(vertices-origin[:3,3])@actual[:3,:3].T+actual[:3,3]
                if not within_pad(placed,pad,radius,margin=.002+FOOTPRINT_MARGIN_M):continue
                poly=inflate_polygon(footprint(placed))
                if blockers(poly,occupancy):continue
                candidate={'offset_xy_m':offset.tolist(),'reserved_polygon_table_m':poly.tolist(),
                           'placement_q_rad':q.tolist(),'pickup_option_index':index,
                           'max_radius_m':float(np.max(np.linalg.norm(poly-pad[:2],axis=1)))}
                # Reserve positions distributed across the pad, rather than six near-identical centers.
                if any(np.linalg.norm(np.asarray(c['offset_xy_m'])-offset)<.003 for c in candidates):continue
                candidates.append(candidate)
                if len(candidates)>=32:break
            if timed_out or len(candidates)>=32:break
        options[key]=candidates
        print(f'패드 {pad_id}: {key} 점유 회피·외곽·IK 통과 위치 {len(candidates)}개',flush=True)
        if not candidates:break
    chosen=jointly_choose(options) if set(options)==set(object_keys) else None
    if chosen is None or not all(key in chosen for key in object_keys):
        ids=[row['id'] for row in occupancy['obstacles']
             if np.linalg.norm(np.maximum(np.maximum(np.min(row['polygon_table_m'],axis=0)-pad[:2],pad[:2]-np.max(row['polygon_table_m'],axis=0)),0))<=radius+.002]
        reason='search_budget' if timed_out else ('occupied_pad' if ids else 'geometry')
        diagnostic={'pad_id':pad_id,'objects':list(object_keys),'reason':reason,'blocked_ids':ids,
                    'retryable':not bool(ids),'search_step_m':.002,'edge_margin_m':.002,'object_gap_m':.002,
                    'scope':'bounded placement candidates; not mathematical proof of impossibility',
                    'reviews':reviews,'occupancy_geometry_digest':occupancy['geometry_digest']}
        (session/f'pad_{pad_id}_fast_search_failed.json').write_text(json.dumps(diagnostic,ensure_ascii=False,indent=2))
        message=f'안전점검 실패: 패드 {pad_id}에 기존 물체를 피해 놓을 후보를 확보하지 못했습니다'
        if ids:message+=f' · 점유 물체 {ids} · 같은 장면 재스캔 반복 안 함'
        if timed_out:message+=' · 배치 탐색 시간 한도 도달'
        raise StaticPlacementRejected(message,retryable=not bool(ids),reason=reason,blocked_ids=ids)
    alternatives={key:[c for c in options[key] if c is not chosen[key]][:5] for key in object_keys} if len(object_keys)==1 else {}
    plan={'version':2,'pad_id':pad_id,'stacking':False,'pad_center_table_m':pad.tolist(),'radius_m':radius,
          'edge_margin_m':.002,'object_gap_m':.002,'search_step_m':.002,
          'placements':chosen,'alternatives':alternatives,'completed':{},'source_scan':str(session),
          'occupied':occupancy['obstacles'],'occupancy_geometry_digest':occupancy['geometry_digest'],
          'scope':'existing placed geometry + exact observed hull + placement IK; full MoveIt path review still required'}
    path.write_text(json.dumps(plan,ensure_ascii=False,indent=2))
    print(f'패드 {pad_id} 빈자리 예약 완료 · 기존 점유 모델 {len(occupancy["obstacles"])}개 유지',flush=True)
    return path


def reserve_single_pad(session, object_key, pad_id, kinematics, **kwargs):
    return reserve_pad(session,[object_key],pad_id,kinematics,**kwargs)
