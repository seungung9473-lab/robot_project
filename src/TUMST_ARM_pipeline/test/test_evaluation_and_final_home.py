"""No hardware: persistent evaluation membership and final-HOME ordering."""
import json,sys
from pathlib import Path
import pytest
R=Path(__file__).resolve().parents[3]/'automation/reconstruction';sys.path.insert(0,str(R))
from success_evaluation import initialize,register,read,write
from batch_completion import finish_batch_home
from test_data_recorder import Trial


def fake_run(root,key,*,kind='natural_language_tidy_demo',physical=True,parent=None,status='complete',finished=True,day=8):
    p=root/'runs'/key;p.mkdir(parents=True)
    summary={'trial_id':key,'started_at':f'2026-11-{day:02d}T23:50:00+09:00','kind':kind,
             'physical_execution_requested':physical,'parent_trial_id':parent,
             'outcome':('complete' if status=='complete' else 'failed') if finished else 'running',
             'finished_at':f'2026-11-{day:02d}T23:55:00+09:00' if finished else None,'duration_s':321 if finished else None}
    write(p/'summary.json',summary)
    write(p/'demo_result.json',{'status':status,'total_seconds':320.98,'date_routes':{'objects':[{}, {}, {}]},'completed_placements':{'a':{},'b':{},'c':{}} if status=='complete' else {'a':{}}})
    return p


def test_first_run_and_exclusions_and_failed_denominator(tmp_path):
    older=fake_run(tmp_path,'older',day=7);first=fake_run(tmp_path,'first');series=initialize(tmp_path,first)
    assert read(series/'trials.json')[0]['number']==1
    for p in [older,fake_run(tmp_path,'child',parent='first'),fake_run(tmp_path,'preview',physical=False),fake_run(tmp_path,'camera',kind='camera_check')]:assert register(p,tmp_path) is None
    fail=fake_run(tmp_path,'second',status='stopped',day=9);register(fail,tmp_path);register(first,tmp_path)
    rows=read(series/'trials.json');assert len(rows)==2 and rows[1]['number']==2
    report=read(series/'summary.json');assert report['logged_success_rate']==.5 and report['observed_success_rate'] is None
    assert rows[0]['observed_success'] is None and rows[0]['objects_completed_logged']==3


def test_limit_50_and_no_duplicate_numbers(tmp_path):
    first=fake_run(tmp_path,'first');series=initialize(tmp_path,first)
    for i in range(1,52):register(fake_run(tmp_path,f'test_{i:02d}',day=9),tmp_path)
    rows=read(series/'trials.json');assert len(rows)==50
    assert [r['number'] for r in rows]==list(range(1,51))
    assert read(series/'summary.json')['remaining_to_register']==0


def test_recorder_automatically_registers_start_and_updates_failure(tmp_path,monkeypatch):
    monkeypatch.delenv('SOARM_TEST_PARENT_ID',raising=False)
    series=initialize(tmp_path,fake_run(tmp_path,'first',day=7))
    trial=Trial('natural_language_tidy_demo',physical=True,root=tmp_path,install_hooks=False)
    assert trial.summary['success_evaluation']['number']==2
    assert read(series/'trials.json')[1]['logged_success'] is None
    result=trial.directory/'demo_result.json';write(result,{'status':'stopped','error':'USB disconnected'})
    trial.attach(result);trial.finish()
    row=read(series/'trials.json')[1]
    assert row['logged_success'] is False and row['error']=='USB disconnected'
    trial.finish();assert len(read(series/'trials.json'))==2


def plan():return {'finish_above':True,'stages':[{'execution_status':'complete'} for _ in range(3)]}


def test_home_once_after_all_three_placements():
    p=plan();calls=[];saved=[]
    def home():
        assert all(s['execution_status']=='complete' for s in p['stages'])
        calls.append('home');return {'success':True,'message':'arrived'}
    finish_batch_home(p,home,lambda:saved.append(p['final_home']['status']))
    finish_batch_home(p,home)
    assert calls==['home'] and saved==['running','complete']


@pytest.mark.parametrize('status',['running','stopped',None])
def test_no_home_after_partial_batch(status):
    p=plan();p['stages'][-1]['execution_status']=status
    with pytest.raises(RuntimeError):finish_batch_home(p,lambda:pytest.fail('must not move'))


def test_failed_home_does_not_report_complete():
    p=plan()
    with pytest.raises(RuntimeError):finish_batch_home(p,lambda:{'success':False,'message':'blocked'})
    assert p['final_home']['status']=='stopped'



def test_excluded_run_cannot_register_again_when_it_finishes(tmp_path):
    first = fake_run(tmp_path, 'first')
    series = initialize(tmp_path, first)
    excluded = fake_run(tmp_path, 'excluded', day=9)
    write(series/'excluded_trials.json', [{'trial_id': 'excluded'}])

    assert register(excluded, tmp_path) is None
    assert [r['trial_id'] for r in read(series/'trials.json')] == ['first']
