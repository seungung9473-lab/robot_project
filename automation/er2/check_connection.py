#!/usr/bin/env python3
"""Gemini Robotics ER2 connection check. No ROS, images or actuator imports."""
import argparse,datetime,getpass,json,os,stat,sys,tempfile,re
from pathlib import Path
from urllib.request import Request,urlopen
from urllib.error import HTTPError,URLError

MODEL=json.loads((Path(__file__).resolve().parent/'model_policy.json').read_text())['primary']
BASE='https://generativelanguage.googleapis.com/v1beta/models/'
CONFIG=Path.home()/'.config/soarm4/gemini.json'
RESULT=Path(__file__).resolve().parent/'connection_status.json'

class APIError(RuntimeError):
    def __init__(self,message,status=None,reason=None):
        super().__init__(message);self.status=status;self.reason=reason

def store_key():
    if not sys.stdin.isatty():raise RuntimeError('키 입력은 별도 터미널에서 실행하세요.')
    key=getpass.getpass('Gemini API 키 입력 (화면에 표시되지 않음): ').strip()
    if not key or len(key)>4096 or any(ord(c)<33 or ord(c)>126 for c in key):raise RuntimeError('키가 비었거나 입력 형식이 올바르지 않습니다.')
    CONFIG.parent.mkdir(parents=True,mode=0o700,exist_ok=True)
    fd,tmp=tempfile.mkstemp(prefix='.gemini-',dir=CONFIG.parent)
    try:
        os.fchmod(fd,0o600)
        with os.fdopen(fd,'w') as f:json.dump({'api_key':key},f)
        os.replace(tmp,CONFIG)
    finally:
        if os.path.exists(tmp):os.unlink(tmp)
    print('키를 본인만 읽을 수 있는 로컬 설정 파일에 저장했습니다.')

def load_key():
    # Same precedence as Google's SDK. Avoid silent use of an older shell key after setup.
    for name in ['GOOGLE_API_KEY','GEMINI_API_KEY']:
        if os.environ.get(name):return os.environ[name],name
    if CONFIG.is_symlink():raise RuntimeError('키 설정 파일이 심볼릭 링크입니다. 일반 파일로 등록하세요.')
    if CONFIG.exists():
        if CONFIG.stat().st_uid!=os.getuid() or stat.S_IMODE(CONFIG.stat().st_mode)&0o077:raise RuntimeError('키 설정 파일 소유자/접근권한을 확인하세요. 권한은600이어야 합니다.')
        key=json.loads(CONFIG.read_text()).get('api_key')
        if isinstance(key,str) and key:return key,'local_private_config'
    return None,None

def request(key,method,path,payload=None):
    headers={'x-goog-api-key':key,'Content-Type':'application/json'}
    data=None if payload is None else json.dumps(payload).encode()
    req=Request(BASE+path,data=data,headers=headers,method=method)
    try:
        with urlopen(req,timeout=45) as response:return json.load(response)
    except HTTPError as exc:
        try:
            body=json.loads(exc.read().decode());error=body.get('error',{});code=error.get('status','HTTP_ERROR');message=str(error.get('message','')).replace(key,'[REDACTED]')
            message=re.sub(r'AIza[0-9A-Za-z_-]{20,}', '[REDACTED]', message)
            message=re.sub(r'(?i)(key=|bearer\s+)[^\s&\"\']+', r'\1[REDACTED]', message)
            reasons=[str(d.get('reason')) for d in error.get('details',[]) if isinstance(d,dict) and d.get('reason')]
            detail=' | '+message[:1500]+(' | reason='+','.join(reasons) if reasons else '')
        except (ValueError,UnicodeError):code='HTTP_ERROR';detail=''
        # Show only the redacted error message/reason, never credentials/headers.
        raise APIError(f'API 응답 HTTP{exc.code} / {code}'+detail,status=exc.code,reason=code) from None
    except URLError:
        raise RuntimeError('Google API 네트워크 연결 실패: 인터넷/DNS/프록시를 확인하세요.') from None

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--setup-key',action='store_true',help='숨김 입력으로 로컬 키 등록 후 연결 시험');args=parser.parse_args()
    result={'checked_at':datetime.datetime.now().astimezone().isoformat(),'model':MODEL,'key_present':False,'model_metadata_access_confirmed':False,'model_access_confirmed':False,'generate_content_accepted':False,'images_uploaded':False,'robot_commands_sent':False,'success':False}
    try:
        if args.setup_key:store_key()
        key,source=load_key()
        if not key:
            print('API 키가 아직 없습니다. --setup-key로 터미널에서 등록하세요.')
            result['status']='missing_api_key';return 2
        result.update(key_present=True,key_source=source)
        from model_router import generate
        payload={'contents':[{'parts':[{'text':'연결 확인용 텍스트 시험입니다. 도구를 호출하거나 로봇 동작을 제안하지 마세요. 연결 성공이라고만 답하세요.'}]}],'generationConfig':{'maxOutputTokens':1024}}
        def validate(response):
            text=''.join(p.get('text','') for c in response.get('candidates',[]) for p in c.get('content',{}).get('parts',[]) if not p.get('thought',False))
            if not text.strip():raise ValueError('No visible text response')
            return text
        response,_,routing=generate(key,payload,validate)
        result.update(model_used=routing['model_used'],fallback_used=routing['fallback_used'],routing=routing)
        candidates=response.get('candidates',[])
        if not candidates:raise RuntimeError('응답 후보가 없습니다. 연결 성공으로 판정하지 않습니다.')
        text=''.join(part.get('text','') for candidate in candidates for part in candidate.get('content',{}).get('parts',[]) if not part.get('thought',False))
        result.update(model_access_confirmed=True,generate_content_accepted=True,response_id=response.get('responseId'),model_version=response.get('modelVersion'),finish_reason=candidates[0].get('finishReason'),usage_metadata=response.get('usageMetadata',{}),visible_text_present=bool(text.strip()))
        if not text.strip():raise RuntimeError('모델 호출은 수락됐지만 표시 응답이 없습니다. 결과 파일의 종료 사유를 확인하세요.')
        result.update(success=True,status='connected')
        print('모델 접근과 실제 텍스트 응답 확인:',routing['model_used'])
        print('카메라 영상 전송과 로봇 동작은 수행하지 않았습니다.')
        return 0
    except (RuntimeError,ValueError,OSError) as exc:
        message=str(exc)
        # Defensive redaction if an unexpected local library exception includes a key.
        if 'key' in locals() and key:message=message.replace(key,'[REDACTED]')
        result.update(status='failed',error=message);print('연결 확인 실패:',message)
        if 'HTTP402' in message or 'prepayment credits are depleted' in message:
            result['cause']='prepay_credits_depleted';print('선불 크레딧이 없거나 소진된 상태입니다. AI Studio에서 해당 프로젝트의 결제 계정을 확인하고 Buy credits로 충전한 뒤 재실행하세요. 키 재등록은 필요하지 않습니다.')
        elif 'free tier is not available' in message:
            result['cause']='project_free_tier_unavailable';print('이 프로젝트는 무료 API 사용 대상이 아닙니다. 같은 프로젝트의 키를 다시 발급해도 해결되지 않습니다. AI Studio에서 해당 프로젝트에 결제를 연결해야 합니다. 유료 호출 요금이 적용됩니다.')
        else:print('HTTP400/401/403: 키·API 제한·프로젝트 권한, HTTP404: 모델 제공 여부, HTTP429: 할당량/결제를 확인하세요.')
        return 1
    finally:
        RESULT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print('점검 결과:',RESULT)

if __name__=='__main__':sys.exit(main())
