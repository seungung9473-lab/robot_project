"""Non-actuating coverage for pre-suction, continuous motion and chained starts."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from continuous_pre_suction import PreSuctionTrigger
from batch_search import require_complete
from normal_contact_trajectory import contact_samples
from turntable_wait import rotation_envelope,review_wait_pose


def test_pump_fires_once_crossing_5mm_during_motion():
    sent=[];future=object()
    trigger=PreSuctionTrigger([0,0,0],[0,0,1],lambda:(sent.append(True),future)[1])
    for z in [.03,.012,.006]:trigger.update([0,0,z])
    assert not sent
    trigger.update([0,0,.0048]);assert sent==[True] and trigger.future is future
    for z in [.003,.001,0]:trigger.update([0,0,z])
    assert len(sent)==1 and trigger.distance==pytest.approx(.0048)


def test_suction_trigger_uses_face_normal_not_world_height():
    fired=[];n=np.array([1,0,1])/math.sqrt(2)
    t=PreSuctionTrigger([1,2,3],n,lambda:fired.append(True) or object())
    t.update(np.array([1,2,3])+n*.006);assert not fired
    t.update(np.array([1,2,3])+n*.004);assert fired


def batch():
    return {'all_stages_ready':True,'finish_above':True,'return_home_between_stages':False,
            'requested_objects':['expired','later','near'],
            'stages':[{'object':'expired','destination':'trash_drop','start_q_rad':[0]*4,'exit_q_rad':[.1]*4},
                      {'object':'later','start_q_rad':[.1]*4,'exit_q_rad':[.2]*4},
                      {'object':'near','start_q_rad':[.2]*4,'exit_q_rad':[.3]*4}]}


def test_chain_includes_pose_after_trash_without_home():
    require_complete(batch())


@pytest.mark.parametrize('mutation',['disconnected','missing','nonfinite','home'])
def test_invalid_chain_stops_before_execution(mutation):
    p=batch()
    if mutation=='disconnected':p['stages'][1]['start_q_rad']=[0]*4
    if mutation=='missing':del p['stages'][0]['exit_q_rad']
    if mutation=='nonfinite':p['stages'][2]['exit_q_rad'][0]=float('nan')
    if mutation=='home':p['return_home_between_stages']=True
    with pytest.raises(RuntimeError):require_complete(p)


def test_full_rotation_envelope_contains_every_rotated_point():
    axis=np.array([1.,2.,.056]);points=np.array([[1.10,2.,.09],[.98,2.07,.11],[1.,1.95,.06]])
    e=rotation_envelope([points],axis,.056)
    for a in np.linspace(0,2*math.pi,721):
        rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        xy=(points[:,:2]-axis[:2])@rot.T+axis[:2]
        assert np.all(np.linalg.norm(xy-np.array(e['center'][:2]),axis=1)<=e['radius'])
    assert e['center'][2]-e['height']/2<.056 and e['center'][2]+e['height']/2>.11


def test_continuous_60mm_lift_has_one_profile_and_bounded_derivatives(monkeypatch):
    import normal_contact_trajectory as c
    def fk(q):
        T=np.eye(4);T[:3,3]=np.asarray(q)[:3];return T
    monkeypatch.setattr(c,'solve_axis',lambda fk,lo,hi,point,axis,seed,attempts:(np.r_[point,0.],None))
    t,q,v,a=contact_samples(fk,[-1]*4,[1]*4,[0]*4,[0,0,.06,0],[.8]*4,[.4]*4)
    assert t[0]==0 and np.all(np.diff(t)>0)
    assert q[-1,2]==pytest.approx(.06)
    assert np.all(np.max(np.abs(v),axis=0)<=.8) and np.all(np.max(np.abs(a),axis=0)<=.4)
    assert np.all(v[2:-2,2]>0)  # No 10mm stop boundaries.


def test_rotation_check_preserves_scanner_executor_and_cleans_scene(tmp_path,monkeypatch):
    import json,yaml,rclpy
    from moveit_msgs.srv import ApplyPlanningScene,GetPlanningScene,GetStateValidity
    source=tmp_path;obj=source/'object';(obj/'analysis').mkdir(parents=True)
    np.savez(obj/'analysis/cloud.npz',points=np.array([[0.,0.,.08],[.1,0.,.10],[0.,.1,.09]]))
    (source/'inclined_pick_plan.json').write_text(json.dumps({'objects':[{'object':'a','session':str(obj)}]}))
    (source/'workcell_snapshot.yaml').write_text(yaml.safe_dump({'turntable':{'axis_point_table_m':[0,0,.056],'top_height_m':.056}}))
    monkeypatch.setattr(rclpy,'spin_until_future_complete',lambda *a,**k:pytest.fail('Scanner executor must not be replaced'))
    class Future:
        def __init__(self,value):self.value=value;self.ready=False
        def done(self):return self.ready
        def result(self):return self.value
    class Node:
        _scan_executor=True
        def __init__(self,valid):self.pending=[];self.operations=[];self.valid=valid
        def create_client(self,typ,path):
            node=self
            class Client:
                def wait_for_service(self,**k):return True
                def call_async(self,req):
                    if typ is GetPlanningScene:value=SimpleNamespace(scene=SimpleNamespace(robot_state=SimpleNamespace(attached_collision_objects=[])))
                    elif typ is ApplyPlanningScene:
                        node.operations.append(req.scene.world.collision_objects[0].operation);value=SimpleNamespace(success=True)
                    else:value=SimpleNamespace(valid=node.valid,contacts=[])
                    f=Future(value);node.pending.append(f);return f
            return Client()
        def destroy_client(self,c):pass
        def spin(self,t):
            for f in self.pending:f.ready=True
    for valid in (True,False):
        n=Node(valid)
        if valid:review_wait_pose(n,[0]*4,source,set())
        else:
            with pytest.raises(RuntimeError,match='안전점검 실패'):review_wait_pose(n,[0]*4,source,set())
        assert [int.from_bytes(v,'little') if isinstance(v,bytes) else int(v) for v in n.operations]==[0,1]  # ADD then REMOVE even if collision rejected.
