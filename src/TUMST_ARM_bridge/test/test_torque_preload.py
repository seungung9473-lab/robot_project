"""No ROS node or hardware: verify holds before any torque write."""
from types import SimpleNamespace
import pytest
from soarm_bridge.servo_bridge import ServoBridge

def fake(fresh=True,corrupt=False):
    writes=[]
    b=SimpleNamespace(read_only=False,pos=[100,200],ids=[1,2],vmin=30,acc=50,torque_on=False,feedback_fresh=lambda:fresh)
    b.bus=SimpleNamespace(sync_goal=lambda goals,acc:writes.append(('hold',goals)),read=lambda sid,addr,n:(0 if corrupt else b.pos[sid-1]).to_bytes(2,'little'),torque=lambda sid,on:writes.append(('torque',sid,on)))
    return b,writes

def test_failed_hold_never_enables_any_joint():
    b,w=fake(corrupt=True)
    with pytest.raises(RuntimeError):ServoBridge.set_torque(b,True)
    assert not any(x[0]=='torque' for x in w)
    assert not b.torque_on

def test_stale_feedback_sends_no_command():
    b,w=fake(fresh=False)
    with pytest.raises(RuntimeError):ServoBridge.set_torque(b,True)
    assert w==[]

def test_verified_hold_precedes_torque():
    b,w=fake();ServoBridge.set_torque(b,True)
    assert w[0][0]=='hold'
    assert w[1:]==[('torque',1,True),('torque',2,True)]
    assert b.torque_on
