"""Fixed HOME endpoints with freely editable motion and vacuum steps."""
import json,math,os,tempfile,uuid,shutil,time
from pathlib import Path
LEGACY_SECTIONS=('before_pick','carry','after_place')
SECTIONS=('route',)
SECTION_LABELS=('홈→흡착','흡착→놓기','놓기→홈')
SLOTS=()
LABELS=('흡착','놓기')
class RecipeStore:
    def __init__(self,path,signature,names,limits):self.path=Path(path).expanduser();self.signature=signature;self.names=names;self.limits=limits
    def validate(self,q):
        if not isinstance(q,list) or len(q)!=len(self.names):raise ValueError('관절값 수가 잘못됐습니다')
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or not lo<=x<=hi for x,(lo,hi) in zip(q,self.limits)):raise ValueError('관절값이 URDF/모터 리미트 밖입니다')
        return q
    def empty(self):return {'version':4,'revision':0,'signature':self.signature,'joint_names':self.names,'segments':{k:[] for k in SECTIONS}}
    def write(self,data):
        self.path.parent.mkdir(parents=True,exist_ok=True);fd,tmp=tempfile.mkstemp(prefix='.route-',dir=self.path.parent)
        try:
            with os.fdopen(fd,'w') as f:json.dump(data,f,ensure_ascii=False,indent=2);f.flush();os.fsync(f.fileno())
            if self.path.exists():shutil.copy2(self.path,self.path.with_suffix('.previous.json'))
            os.replace(tmp,self.path)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)
    def load(self,complete=False):
        data=json.loads(self.path.read_text()) if self.path.exists() else self.empty()
        compatible=data.get('signature')==self.signature and data.get('joint_names')==self.names
        if not compatible:raise ValueError('모델/엔코더 보정이 변경됐습니다. 전체 초기화 후 자세를 다시 저장하세요 (기존 파일 백업 보존)')
        if data.get('version') in (1,2,3):
            version=data['version'];shutil.copy2(self.path,self.path.with_name(self.path.stem+f'.v{version}-backup-'+str(time.time_ns())+'.json'))
            items=[]
            if version==1:
                anchors=data.get('waypoints',{});segments={k:[] for k in LEGACY_SECTIONS}
            else:anchors=data.get('anchors',{});segments=data['segments']
            for section,anchor in [('before_pick','pick'),('carry','place'),('after_place',None)]:
                if version in (1,2) and section in ('carry','after_place'):items.append({'id':uuid.uuid4().hex,'kind':'grip' if section=='carry' else 'release'})
                for old in segments[section]:
                    item=dict(old);item.setdefault('kind','move');items.append(item)
                if anchor in anchors:items.append({'id':uuid.uuid4().hex,'kind':'move','q':anchors[anchor]})
            data={'version':4,'revision':data.get('revision',0)+1,'signature':self.signature,'joint_names':self.names,'segments':{'route':items}}
            self.write(data)
        if data.get('version')!=4 or not isinstance(data.get('segments'),dict) or set(data['segments'])!=set(SECTIONS):raise ValueError('잘못된 경로 파일')
        ids=set()
        for group in SECTIONS:
            if not isinstance(data['segments'][group],list):raise ValueError('잘못된 중간 경로 목록')
            for item in data['segments'][group]:
                if not isinstance(item,dict) or not isinstance(item.get('id'),str) or item['id'] in ids:raise ValueError('잘못된 중간 자세 ID')
                ids.add(item['id'])
                if item.get('kind')=='move':self.validate(item.get('q'))
                elif item.get('kind') not in ('grip','release'):raise ValueError('잘못된 경로 단계 종류')
        if complete:self.validate_actions(data)
        return data
    def validate_actions(self,data):
        holding=False;pairs=0
        for group in SECTIONS:
            for item in data['segments'][group]:
                if item['kind']=='grip':
                    if holding:raise ValueError('이미 흡착 중인데 흡착 단계가 중복됩니다')
                    holding=True
                elif item['kind']=='release':
                    if not holding:raise ValueError('흡착 단계보다 먼저 놓기 단계가 있습니다')
                    holding=False;pairs+=1
        if holding:raise ValueError('종료 홈 전에 놓기 단계를 추가하세요')
        if pairs==0:raise ValueError('흡착 단계와 놓기 단계를 경로에 추가하세요')
    def save(self,slot,q):return self.edit({'action':'save_anchor','anchor':slot},q)
    def edit(self,cmd,q=None):
        action=cmd.get('action')
        try:data=self.load()
        except (ValueError,KeyError,TypeError):
            if action!='clear':raise
            if self.path.exists():shutil.copy2(self.path,self.path.with_name(self.path.stem+'.invalid-backup-'+str(time.time_ns())+'.json'))
            data=self.empty()
        revision=data.get('revision',0)
        if 'revision' in cmd and cmd['revision']!=revision:raise ValueError('목록이 변경됐습니다. 새 목록에서 다시 선택하세요')
        if action=='clear':data=self.empty()
        else:
            section=cmd.get('section')
            if section not in SECTIONS:raise ValueError('중간 경로 구간을 선택하세요')
            items=data['segments'][section];index=cmd.get('index')
            if type(index) is not int:raise ValueError('잘못된 중간 자세 번호')
            if action in ('insert','insert_grip','insert_release'):
                if not 0<=index<=len(items):raise ValueError('추가 위치가 변경됐습니다. 목록을 다시 확인하세요')
                items.insert(index,{'id':uuid.uuid4().hex,'kind':'move','q':list(self.validate(q))} if action=='insert' else {'id':uuid.uuid4().hex,'kind':action.removeprefix('insert_')})
            elif action in ('replace','delete','move'):
                if not 0<=index<len(items):raise ValueError('선택한 중간 자세가 없습니다')
                if action=='replace':
                    if items[index]['kind']!='move':raise ValueError('흡착/놓기 단계는 위치를 저장하지 않습니다')
                    items[index]['q']=list(self.validate(q))
                elif action=='delete':items.pop(index)
                else:
                    delta=cmd.get('delta')
                    if type(delta) is not int or delta not in (-1,1) or not 0<=index+delta<len(items):raise ValueError('시작/종료 홈 밖으로 이동할 수 없습니다')
                    items[index],items[index+delta]=items[index+delta],items[index]
            else:raise ValueError('알 수 없는 편집 명령')
        data['revision']=revision+1
        self.write(data);return data
    def route(self,home,data=None,complete=True):
        data=self.load(complete) if data is None else data;self.validate(home)
        steps=[{'key':'home_start','label':'홈 (시작·고정)','q':list(home),'kind':'home','section':'route','index':-1,'action':None}]
        for i,item in enumerate(data['segments']['route']):
            is_move=item['kind']=='move';label_item=f'자세 {sum(x["kind"]=="move" for x in data["segments"]["route"][:i+1])}' if is_move else ('흡착 ON · 압력 확인' if item['kind']=='grip' else '놓기 · 펌프 OFF/밸브 해제')
            steps.append({'key':item['id'],'label':label_item,'q':item.get('q'),'kind':'waypoint' if is_move else 'action','section':'route','index':i,'action':None if is_move else item['kind']})
        steps.append({'key':'home_end','label':'홈 (종료·고정)','q':list(home),'kind':'home','section':'route','index':len(data['segments']['route'])-1,'action':None})
        return steps

def run_sequence(steps,move,grip,release,guard):
    for step in steps:
        guard()
        if step['kind']!='action':move(step)
        if step['action']=='grip':guard();grip()
        elif step['action']=='release':guard();release()
