"""Local experiment records. No ROS, network, or hardware commands."""
import atexit,csv,datetime,fcntl,json,os,sys,time,uuid
from pathlib import Path
from zoneinfo import ZoneInfo

DEFAULT_ROOT=(Path(__file__).resolve().parent.parent/'test_data').resolve()
TZ=ZoneInfo('Asia/Seoul')

def stamp():return datetime.datetime.now(TZ).isoformat()

def atomic_json(path,value):
    path=Path(path);tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n');tmp.replace(path)

def refresh_index(root=DEFAULT_ROOT):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    with (root/'.index.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        rows=[]
        for p in sorted((root/'runs').glob('*/summary.json')):
            try:r=json.loads(p.read_text())
            except (OSError,ValueError):continue
            rows.append({k:r.get(k,'') for k in ['trial_id','parent_trial_id','started_at','finished_at','kind','physical_execution_requested','outcome','error','source']})
        tmp=root/'index.csv.tmp'
        with tmp.open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=['trial_id','parent_trial_id','started_at','finished_at','kind','physical_execution_requested','outcome','error','source']);w.writeheader();w.writerows(rows)
        tmp.replace(root/'index.csv')

class Trial:
    def __init__(self,kind,physical=False,source=None,root=None,install_hooks=True):
        self.root=Path(root or DEFAULT_ROOT);self.id=datetime.datetime.now(TZ).strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self.directory=self.root/'runs'/self.id;self.directory.mkdir(parents=True)
        self.started=time.monotonic();self.last_sample={};self.closed=False;self.attachments=[]
        self.events=(self.directory/'events.jsonl').open('a',buffering=1)
        self.telemetry=(self.directory/'telemetry.jsonl').open('a',buffering=65536)
        self.summary={'schema_version':1,'trial_id':self.id,'parent_trial_id':os.environ.get('SOARM_TEST_PARENT_ID'),'started_at':stamp(),'kind':kind,'physical_execution_requested':bool(physical),'source':str(Path(source).resolve()) if source else None,'outcome':'running','error':None,'telemetry_max_hz_per_signal':10,'retrospective':False,'artifacts':[]}
        atomic_json(self.directory/'summary.json',self.summary);self.event('started');refresh_index(self.root)
        self.track_evaluation()
        if install_hooks:
            os.environ['SOARM_TEST_PARENT_ID']=self.id
            previous=sys.excepthook
            def hook(typ,value,tb):
                self.summary.update(outcome='interrupted' if issubclass(typ,KeyboardInterrupt) else 'failed',error=typ.__name__+': '+str(value));self.finish();previous(typ,value,tb)
            sys.excepthook=hook;atexit.register(self.finish)

    def track_evaluation(self):
        from success_evaluation import register
        registration=register(self.directory,self.root)
        if registration:
            self.summary['success_evaluation']=registration
            atomic_json(self.directory/'summary.json',self.summary)

    def event(self,name,**data):
        if not self.closed:self.events.write(json.dumps({'at':stamp(),'elapsed_s':time.monotonic()-self.started,'event':name,**data},ensure_ascii=False,allow_nan=False)+'\n')

    def sample(self,signal,**data):
        now=time.monotonic()
        if self.closed or now-self.last_sample.get(signal,-1e9)<.1:return
        self.last_sample[signal]=now
        self.telemetry.write(json.dumps({'at':stamp(),'elapsed_s':now-self.started,'signal':signal,**data},ensure_ascii=False,allow_nan=False)+'\n')

    def attach(self,path):
        p=str(Path(path).resolve())
        if p not in self.attachments:self.attachments.append(p)

    def finish(self,outcome=None,error=None):
        if self.closed:return
        if outcome:self.summary['outcome']=outcome
        if error:self.summary['error']=error
        snapshots={}
        for path in self.attachments:
            p=Path(path)
            if not p.exists():continue
            self.summary['artifacts'].append(path)
            if p.suffix=='.json' and p.stat().st_size<8_000_000:
                try:snapshots[path]=json.loads(p.read_text())
                except (OSError,ValueError):pass
        if self.summary['outcome']=='running':
            values=list(snapshots.values())
            failures=[r for r in values if r.get('success') is False or r.get('status')=='stopped' or r.get('execution_status')=='stopped' or r.get('all_stages_ready') is False]
            if failures:self.summary.update(outcome='failed',error=failures[-1].get('error','See linked result'))
            elif any(r.get('status')=='complete_with_unreadable_objects' for r in values):self.summary['outcome']='partial'
            elif any(r.get('success') is True or r.get('status')=='complete' or r.get('execution_status')=='complete' for r in values):self.summary['outcome']='complete'
            elif any(r.get('all_stages_ready') is True for r in values):self.summary['outcome']='planned'
            else:self.summary['outcome']='finished_without_verified_result'
        self.summary.update(finished_at=stamp(),duration_s=time.monotonic()-self.started)
        self.event('finished',outcome=self.summary['outcome'],error=self.summary['error'])
        atomic_json(self.directory/'results.json',snapshots);atomic_json(self.directory/'summary.json',self.summary)
        self.telemetry.close();self.events.close();self.closed=True;refresh_index(self.root)
        try:self.track_evaluation()
        except Exception as exc:print('평가 집계 갱신 실패 · 원본 기록 보존: '+str(exc),file=sys.stderr)

def start_trial(kind,physical=False,source=None):return Trial(kind,physical,source)

def import_existing(source,kind,root=DEFAULT_ROOT):
    """Index historical result files without inventing missing telemetry."""
    source=Path(source).resolve();root=Path(root)
    import hashlib
    key='historical_'+hashlib.sha256(str(source).encode()).hexdigest()[:20];folder=root/'runs'/key
    if (folder/'summary.json').exists():return False
    data=json.loads(source.read_text());folder.mkdir(parents=True,exist_ok=True)
    success=data.get('success');state=data.get('execution_status',data.get('status'))
    outcome='complete' if success is True or state=='complete' else ('partial' if state=='complete_with_unreadable_objects' else ('failed' if success is False or state=='stopped' else 'unknown'))
    atomic_json(folder/'summary.json',{'schema_version':1,'trial_id':key,'parent_trial_id':None,'started_at':None,'finished_at':None,'kind':kind,'physical_execution_requested':kind=='physical_execution','source':str(source),'outcome':outcome,'error':data.get('error'),'retrospective':True,'indexed_at':stamp(),'original_file_modified_at':datetime.datetime.fromtimestamp(source.stat().st_mtime,TZ).isoformat(),'telemetry_not_reconstructed':True,'artifacts':[str(source)]})
    atomic_json(folder/'results.json',{str(source):data});return True

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--import-existing',action='store_true');a=p.parse_args();count=0
    if a.import_existing:
        base=Path(__file__).resolve().parent
        for pattern,kind in [('expiry_cycles/**/placement_left_*/execution.json','physical_execution'),('expiry_cycles/**/batch_plan.json','batch'),('expiry_cycles/*/cycle.json','expiry_cycle')]:
            for source in base.glob(pattern):count+=int(import_existing(source,kind))
    refresh_index();print('Indexed',count,'existing results:',DEFAULT_ROOT/'index.csv')
