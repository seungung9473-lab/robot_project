"""Bounded ER2 -> Flash failover. Inference only; never executes robot actions."""
import datetime,json,socket
from pathlib import Path
from check_connection import request,APIError
ROOT=Path(__file__).resolve().parent
class InvalidModelOutput(RuntimeError):pass
class BlockedModelOutput(RuntimeError):pass

def policy():
    p=json.loads((ROOT/'model_policy.json').read_text())
    if p['primary']!='gemini-robotics-er-2-preview' or p['fallback']!='gemini-3.8-flash':raise ValueError('Unexpected model routing policy')
    if p['max_requests_per_task']!=2:raise ValueError('Only two bounded requests supported')
    return p

def fallback_reason(exc,p):
    if isinstance(exc,APIError):
        if exc.status in p['never_fallback_http_statuses']:return None
        if exc.status in p['fallback_http_statuses']:return 'HTTP'+str(exc.status)
    if isinstance(exc,(TimeoutError,socket.timeout)) and p['fallback_on_timeout']:return 'timeout'
    if isinstance(exc,InvalidModelOutput) and p['fallback_on_invalid_output']:return 'invalid_output'
    return None

def generate(key,payload,validator,transport=None,test_context=None):
    p=policy();transport=transport or request;attempts=[];models=[p['primary']]+([p['fallback']] if p['fallback_enabled'] else []);route={'primary_model':p['primary'],'attempts':attempts,'test_context':test_context,'robot_commands_sent':False}
    try:
        for index,model in enumerate(models):
            try:
                response=transport(key,'POST',model+':generateContent',payload)
                finishes=[c.get('finishReason') for c in response.get('candidates',[])]
                if response.get('promptFeedback',{}).get('blockReason') or any(f not in (None,'STOP','MAX_TOKENS') for f in finishes):raise BlockedModelOutput('Model output blocked; no fallback or execution')
                try:value=validator(response)
                except (ValueError,KeyError,TypeError) as exc:raise InvalidModelOutput('Response failed local schema/coordinate validation') from None
                version=response.get('modelVersion')
                if not version or not (version==model or version.startswith(model+'-')):raise InvalidModelOutput('Returned model identity not verified')
                attempts.append({'model':model,'status':'success','model_version':version});route.update(success=True,model_used=model,fallback_used=index>0)
                return response,value,route
            except (APIError,InvalidModelOutput,BlockedModelOutput,TimeoutError,OSError,RuntimeError) as exc:
                why=fallback_reason(exc,p);attempts.append({'model':model,'status':'failed','http_status':getattr(exc,'status',None),'category':why or 'not_eligible_for_fallback'})
                if index==0 and len(models)>1 and why:
                    route['fallback_reason']=why;print('ER2 사용 불가 ('+why+') → gemini-3.8-flash로 전환',flush=True);continue
                route.update(success=False,fallback_used=index>0)
                raise
        raise RuntimeError('No inference model available')
    finally:
        log={'time':datetime.datetime.now().astimezone().isoformat(),**route}
        with (ROOT/'routing_history.jsonl').open('a') as f:f.write(json.dumps(log,ensure_ascii=False)+'\n')
