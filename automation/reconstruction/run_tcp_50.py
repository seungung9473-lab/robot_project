"""50 bounded local motions; encoder-only verification, never updates calibration."""
import json,subprocess,sys,os
from pathlib import Path
import numpy as np
from test_data_recorder import start_trial,atomic_json
r=start_trial('tcp_50_encoder_trials',physical=True,source=__file__)
results=[]
try:
    # Obtain fresh feedback through a plan-only invocation; this sends no motor goals.
    cmd=['ros2','run','soarm4_workcell','tcp_error_test']
    p=subprocess.run(cmd+['--delta-mm','0','0','0'],capture_output=True,text=True)
    (r.directory/'baseline.log').write_text(p.stdout+p.stderr)
    if p.returncode:raise RuntimeError('Baseline planning failed: '+p.stderr[-1000:])
    baseline=Path(p.stdout.strip().splitlines()[-1]);r.attach(baseline);d=json.loads(baseline.read_text());center=np.array(d['initial_table_m']);seed=np.radians(d['start_deg'])
    from soarm4_workcell.cup_goal import Arm,tcp_point
    from soarm4_workcell.core import load,matrix
    from scipy.optimize import least_squares
    arm=Arm();wc=load(Path.home()/'robot_ws/src/soarm4_workcell/config/workcell.yaml');W=matrix(wc['T_table_world']);tcp=tcp_point(wc)
    def fk(q):
        T=W@arm.fk(q);T[:3,3]=(T@tcp)[:3];return T
    axis=fk(seed)[:3,0];targets=[]
    # Vertical positive displacement avoids current J2/J3 limits. Every segment <=3mm.
    for i in range(50):
        height=(i%10+1)*.0003 if (i//10)%2==0 else (10-i%10)*.0003
        target=center+[0,0,height]
        fit=least_squares(lambda q:np.r_[(fk(q)[:3,3]-target)*10,(fk(q)[:3,0]-axis)*.3],np.clip(seed,arm.lo,arm.hi),bounds=(arm.lo,arm.hi),max_nfev=500)
        if np.linalg.norm(fk(fit.x)[:3,3]-target)>.0001 or np.linalg.norm(fk(fit.x)[:3,0]-axis)>.005:raise RuntimeError('Offline IK failed at '+str(i+1))
        targets.append(target.tolist())
    atomic_json(r.directory/'targets.json',{'center_table_m':center.tolist(),'targets_table_m':targets,'scope':'50 local vertical moves, 0.3-3mm; not workspace calibration'})
    resume=os.environ.get('SOARM_TCP_RESUME')
    if resume:
        previous=Path(resume);old=json.loads((previous/'targets.json').read_text());targets=old['targets_table_m'];results=json.loads((previous/'progress.json').read_text())['results'];r.attach(previous/'progress.json');r.event('resume_after_feedback_refresh_fix',previous=str(previous),completed=len(results))
        atomic_json(r.directory/'targets.json',old)
    for i,t in enumerate(targets):
        if i<len(results):continue
        r.event('motion_start',index=i+1,target_table_m=t)
        p=subprocess.run(cmd+['--xyz',*[str(x) for x in t],'--velocity','.5','--acceleration','.1','--execute'],capture_output=True,text=True)
        (r.directory/f'motion_{i+1:02}.log').write_text(p.stdout+p.stderr)
        if p.returncode:raise RuntimeError('Motion '+str(i+1)+' failed: '+(p.stderr+p.stdout)[-1500:])
        path=Path(p.stdout.strip().splitlines()[-1]);r.attach(path);result=json.loads(path.read_text());results.append(result)
        atomic_json(r.directory/'progress.json',{'completed':len(results),'requested':50,'results':results})
        print('Completed',i+1,'/50',flush=True)
    errors=np.array([x['joint_error_deg'] for x in results]);tcp_errors=np.array([x['model_tcp_error_mm'] for x in results])
    summary={'completed':len(results),'velocity_scaling':.5,'acceleration_scaling':.1,'joint_mean_error_deg':errors.mean(0).tolist(),'joint_rmse_deg':np.sqrt((errors**2).mean(0)).tolist(),'joint_max_abs_deg':np.abs(errors).max(0).tolist(),'model_tcp_rmse_mm':np.sqrt((tcp_errors**2).mean(0)).tolist(),'model_tcp_max_norm_mm':float(np.linalg.norm(tcp_errors,axis=1).max()),'calibration_changed':False,'physical_tip_measured':False,'scope':'Local vertical encoder tracking only; no geometry calibration'}
    atomic_json(r.directory/'summary_metrics.json',summary);r.attach(r.directory/'summary_metrics.json');r.finish('complete');print(json.dumps(summary,indent=2));print(r.directory)
except BaseException as e:
    r.event('stopped',completed=len(results),reason=str(e));r.finish('failed',str(e));raise
