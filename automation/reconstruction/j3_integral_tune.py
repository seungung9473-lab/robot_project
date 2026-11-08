"""J3-only bounded integral trial. Requires supported arm and exclusive bus.
Never changes zeros, angle limits, P/D, deadbands, protections, or other motors.
"""
import json,time,subprocess,sys
from pathlib import Path
from soarm_bridge.sts_bus import STSBus
from test_data_recorder import start_trial,atomic_json

REG_I=23;REG_I_LIMIT=25

def read_checked(bus,addr,count=1):
    for _ in range(3):
        data=bus.read(3,addr,count)
        if data is not None and len(data)==count:return list(data)
    raise RuntimeError('J3 read failed at '+str(addr))

def write_checked(bus,addr,value):
    bus.write(3,addr,[value]);time.sleep(.03)
    if read_checked(bus,addr)!=[value]:raise RuntimeError('J3 write verification failed at '+str(addr))

def program_integral(bus,before,i_value=1,limit_value=4):
    """No torque control here. Verify off, constrain writes, and rollback on failure."""
    if read_checked(bus,40)!=[0]:raise RuntimeError('J3 torque must be OFF')
    if (i_value,limit_value) not in [(1,4),(0,0)]:raise ValueError('Only bounded trial or original settings supported')
    lock=read_checked(bus,55)[0];write_checked(bus,55,0)
    try:
        try:
            if i_value==0:
                write_checked(bus,REG_I,0);write_checked(bus,REG_I_LIMIT,0)
            else:
                write_checked(bus,REG_I_LIMIT,limit_value);write_checked(bus,REG_I,i_value)
            after=read_checked(bus,21,7);wanted=list(before);wanted[2]=i_value;wanted[4]=limit_value
            if after!=wanted:raise RuntimeError('Unexpected PID/deadband register change')
        except BaseException:
            write_checked(bus,REG_I,before[2]);write_checked(bus,REG_I_LIMIT,before[4]);raise
    finally:write_checked(bus,55,lock)
    return after

def main():
    import argparse
    parser=argparse.ArgumentParser();group=parser.add_mutually_exclusive_group();group.add_argument('--apply',action='store_true');group.add_argument('--restore',action='store_true');args=parser.parse_args()
    writing=args.apply or args.restore;requested=(0,0) if args.restore else (1,4)
    if subprocess.run(['fuser','/dev/servo_bus'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0:raise RuntimeError('Servo bus is in use; refuse concurrent access')
    trial=start_trial('j3_integral_original_restore' if args.restore else 'j3_bounded_integral_hardware_trial',physical=writing,source=__file__);bus=STSBus('/dev/servo_bus',1000000);before=None;changed=False;armed=False
    try:
        before=read_checked(bus,21,7);torque=read_checked(bus,40)[0];lock=read_checked(bus,55)[0];mode=read_checked(bus,33)[0];raw=read_checked(bus,56,2);position=raw[0]+256*raw[1]
        snapshot={'motor_id':3,'registers_21_27':before,'torque_enabled':torque,'eeprom_lock':lock,'operating_mode':mode,'position_counts':position,'requested_I':requested[0],'requested_integral_limit':requested[1]}
        p=trial.directory/'before.json';atomic_json(p,snapshot);trial.attach(p)
        if not writing:trial.finish('read_only_complete');print(p);return
        if before!=([32,32,1,0,4,0,0] if args.restore else [32,32,0,0,0,0,0]) or mode!=0:raise RuntimeError('J3 settings differ from inspected baseline; no change applied')
        if not 0<=position<=2047:raise RuntimeError('J3 position outside configured motor range')
        write_checked(bus,40,0);armed=True;trial.event('j3_torque_off',arm_support_confirmed_by_user=True)
        after=program_integral(bus,before,*requested);changed=True;trial.event('registers_verified',before=before,after=after)
        raw=read_checked(bus,56,2);now=raw[0]+256*raw[1]
        if abs(now-position)>8:raise RuntimeError('Supported J3 moved >8 counts; torque left OFF')
        # Preload a current-position hold while torque is off, so old goals cannot cause a jump.
        bus.sync_goal({3:(now,30)},acc=50);time.sleep(.05)
        if read_checked(bus,42,2)!=[now&255,now>>8]:raise RuntimeError('Current-position hold verification failed')
        if torque:
            write_checked(bus,40,1);trial.event('j3_torque_restored_current_position',hold_counts=now)
        positions=[];end=time.monotonic()+1
        while time.monotonic()<end:
            raw=read_checked(bus,56,2);pcount=raw[0]+256*raw[1];positions.append(pcount);trial.sample('j3_hold_position',count=pcount)
            if abs(pcount-now)>4:
                write_checked(bus,40,0);raise RuntimeError('Hold movement >4 counts; J3 torque OFF')
            time.sleep(.03)
        out={'restored_original_settings':args.restore,'motor_id':3,'before_registers_21_27':before,'after_registers_21_27':after,'torque_restored_to_original_state':torque,'eeprom_lock':read_checked(bus,55)[0],'hold_target_counts':now,'hold_min_counts':min(positions),'hold_max_counts':max(positions),'other_joint_torque_changed':False,'other_joint_goals_sent':False,'pose_change_requested':False,'performance_improvement_verified':False}
        p=trial.directory/'result.json';atomic_json(p,out);trial.attach(p);trial.finish('settings_applied_hold_only');print(json.dumps(out,indent=2));print(trial.directory)
    except BaseException as exc:
        # Never re-enable torque on an unknown pose. Restore original I settings while off.
        if writing and before is not None and armed:
            try:
                write_checked(bus,40,0)
                if changed:
                    oldlock=read_checked(bus,55)[0];write_checked(bus,55,0);write_checked(bus,REG_I,before[2]);write_checked(bus,REG_I_LIMIT,before[4]);write_checked(bus,55,oldlock)
                trial.event('failed_safe',j3_torque_off=True,original_integral_restored=changed)
            except Exception as rollback:trial.event('rollback_failed',error=str(rollback))
        trial.finish('failed',str(exc));raise
    finally:bus.close()

if __name__=='__main__':main()
