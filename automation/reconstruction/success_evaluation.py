"""Local, persistent 50-run evaluation; never commands hardware or alters raw runs."""
import csv,datetime,fcntl,json,os,subprocess,uuid
from pathlib import Path

KINDS={'natural_language_tidy_demo','expiry_cycle'}


def read(path,default=None):
    path=Path(path)
    return json.loads(path.read_text()) if path.exists() else default


def write(path,value):
    path=Path(path);tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n');tmp.replace(path)


def result_row(directory,summary):
    result=read(directory/'demo_result.json',{})
    if not result:
        for artifact in summary.get('artifacts',[]):
            if Path(artifact).name=='cycle.json':result=read(artifact,{});break
    scan=result.get('scan_session',result.get('session'))
    completed=result.get('completed_placements')
    if completed is None and scan:completed=read(Path(scan)/'completed_placements.json',{})
    completed=completed or {}
    routes=result.get('date_routes',{}).get('objects')
    if routes is None and scan:routes=read(Path(scan)/'expiry_route_report.json',{}).get('objects')
    duration=result.get('total_seconds',summary.get('duration_s'))
    outcome=summary.get('outcome','running');finished=bool(summary.get('finished_at'))
    success=None if not finished else outcome=='complete' and result.get('status')=='complete'
    return {'trial_id':summary['trial_id'],'run_directory':str(directory.resolve()),
            'started_at':summary['started_at'],'finished_at':summary.get('finished_at'),
            'outcome':outcome,'logged_success':success,'duration_s':duration,
            'objects_total':len(routes) if routes is not None else None,
            'objects_completed_logged':len(completed) if result else None,
            'error':result.get('error') or summary.get('error'),
            'gpu':result.get('sam2_runtime',{}),'stages':result.get('stages',[]),
            'raw_summary':str(directory/'summary.json'),'raw_result':str(directory/'demo_result.json') if (directory/'demo_result.json').exists() else None}


def refresh(series,config,rows):
    ended=[r for r in rows if r['finished_at']]
    successes=sum(r['logged_success'] is True for r in ended)
    observed=[r for r in rows if r.get('observed_success') is not None]
    report={'series_id':config['series_id'],'target_runs':config['target_runs'],
            'registered_runs':len(rows),'finished_runs':len(ended),'running_or_unfinished_runs':len(rows)-len(ended),
            'remaining_to_register':max(0,config['target_runs']-len(rows)),
            'logged_successes':successes,'logged_failures_or_interruptions':len(ended)-successes,
            'logged_success_rate':successes/len(ended) if ended else None,
            'observed_results_count':len(observed),
            'observed_success_rate':sum(r['observed_success'] is True for r in observed)/len(observed) if observed else None,
            'note':'Software completion and human-observed physical success are separate. Failures/interruptions stay in the denominator.'+(' Historical run #1 predates final HOME.' if config.get('anchor_trial_id') else '')}
    write(series/'summary.json',report)
    fields=['number','trial_id','started_at','finished_at','outcome','logged_success','observed_success','duration_s','objects_total','objects_completed_logged','git_revision_at_registration','error','run_directory']
    tmp=series/'trials.csv.tmp'
    with tmp.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    tmp.replace(series/'trials.csv')
    text=f"50회 성공률 평가: {len(rows)}/{config['target_runs']}회 등록\n\n로그상 완료: {successes}/{len(ended)}회. 실제 결과 확인: {len(observed)}회.\n실패·중단은 제외하지 않으며, 미종료 실행은 결과 대기입니다.\n\n"
    for row in rows:text+=f"{row['number']:02d}. {row['trial_id']} | {row['outcome']} | {row['duration_s']}초 | 물체 {row['objects_completed_logged']}/{row['objects_total']} | 실제 결과: {row.get('observed_success')}\n"
    (series/'README_ko.md').write_text(text)


def register(directory,root):
    directory=Path(directory).resolve();root=Path(root).resolve()
    config=read(root/'evaluations/active.json')
    if not config or not config.get('enabled'):return None
    summary=read(directory/'summary.json',{})
    if summary.get('kind') not in KINDS or not summary.get('physical_execution_requested') or summary.get('parent_trial_id'):return None
    if datetime.datetime.fromisoformat(summary['started_at'])<datetime.datetime.fromisoformat(config['anchor_started_at']):return None
    series=root/'evaluations'/config['series_id'];series.mkdir(parents=True,exist_ok=True)
    with (series/'.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        rows=read(series/'trials.json',[]);existing=next((r for r in rows if r['trial_id']==summary['trial_id']),None)
        if existing is None:
            if len(rows)>=config['target_runs']:return None
            revision=subprocess.run(['git','rev-parse','HEAD'],cwd=Path(__file__).resolve().parents[2],capture_output=True,text=True).stdout.strip() or None
            existing={'number':len(rows)+1,'trial_id':summary['trial_id'],'observed_success':None,
                      'git_revision_at_registration':revision if not summary.get('finished_at') else None,
                      'git_revision_note':'Historical revision not inferred from current checkout' if summary.get('finished_at') else None}
            rows.append(existing)
            print(f"[50회 평가] {existing['number']}/{config['target_runs']}회차 자동 등록 · {summary['trial_id']}",flush=True)
        existing.update(result_row(directory,summary))
        write(series/'trials.json',rows);refresh(series,config,rows)
        return {'series_id':config['series_id'],'number':existing['number'],'target_runs':config['target_runs']}


def initialize(root,first):
    root=Path(root).resolve();first=Path(first).resolve();summary=read(first/'summary.json')
    if not summary or summary['kind'] not in KINDS or not summary.get('physical_execution_requested'):raise ValueError('First run must be a recorded full physical cycle')
    folder=root/'evaluations';folder.mkdir(parents=True,exist_ok=True)
    active=folder/'active.json'
    config={'version':1,'series_id':'table_clear_success_50_from_'+summary['trial_id'],'enabled':True,
            'target_runs':50,'anchor_trial_id':summary['trial_id'],'anchor_started_at':summary['started_at'],
            'count_policy':'one top-level physical full-cycle invocation; success/failure/interruption included; children/previews excluded; no automatic robot execution',
            'observed_success_policy':'human confirmation; never inferred from software completion',
            'configuration_policy':'record revision for each new run; first run predates final HOME change'}
    previous=read(active)
    if previous and previous['series_id']!=config['series_id']:raise RuntimeError('Another evaluation already active; refusing to overwrite')
    write(active,config)
    eligible=[]
    for directory in (root/'runs').iterdir():
        s=read(directory/'summary.json',{})
        if s.get('started_at') and s.get('kind') in KINDS and s.get('physical_execution_requested') and not s.get('parent_trial_id') and s['started_at']>=config['anchor_started_at']:eligible.append((s['started_at'],directory))
    for _,directory in sorted(eligible):register(directory,root)
    return root/'evaluations'/config['series_id']


def record_observation(root,number,success,note):
    """사람이 확인한 결과를 회차에 기록한다. 이미 기록된 값은 덮어쓰지 않는다."""
    if not isinstance(success,bool) or not note:raise ValueError('success(bool)와 사유(note)가 필요합니다')
    root=Path(root).resolve();config=read(root/'evaluations/active.json')
    series=root/'evaluations'/config['series_id']
    with (series/'.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        rows=read(series/'trials.json',[]);row=next((r for r in rows if r['number']==number),None)
        if row is None:raise ValueError(f'{number}회차 없음')
        if row.get('observed_success') is not None:raise ValueError(f'{number}회차는 이미 사람 확인 결과가 있습니다')
        row['observed_success']=success;row['observed_note']=note
        row['observed_recorded_at']=datetime.datetime.now().astimezone().isoformat()
        write(series/'trials.json',rows);refresh(series,config,rows)
        return row


if __name__=='__main__' and len(__import__('sys').argv)>=5 and __import__('sys').argv[1]=='observe':
    import sys
    print(record_observation(Path(__file__).resolve().parents[1]/'test_data',int(sys.argv[2]),sys.argv[3]=='success',sys.argv[4]))


def restart_series(root):
    """현재 평가를 삭제하지 않고 archive로 옮긴 뒤, 지금 시각 이후 첫 전체 실행부터 1회차로 새로 시작한다."""
    root=Path(root).resolve();folder=root/'evaluations';active=folder/'active.json';old=read(active)
    now=datetime.datetime.now().astimezone();stamp=now.strftime('%Y%m%d_%H%M%S')
    if old:
        archive=folder/('archive_'+stamp);archive.mkdir(parents=True)
        (folder/old['series_id']).rename(archive/old['series_id']);active.rename(archive/'active.json')
    config={'version':1,'series_id':'table_clear_success_50_restart_'+stamp,'enabled':True,'target_runs':50,'anchor_trial_id':None,
            'anchor_started_at':now.isoformat(),
            'count_policy':'one top-level physical full-cycle invocation; success/failure/interruption included; children/previews excluded; no automatic robot execution',
            'observed_success_policy':'human confirmation; never inferred from software completion',
            'configuration_policy':'record git revision for each new run','restarted_from':old['series_id'] if old else None}
    write(active,config);series=folder/config['series_id'];series.mkdir(parents=True,exist_ok=True)
    write(series/'trials.json',[]);refresh(series,config,[])
    return config
