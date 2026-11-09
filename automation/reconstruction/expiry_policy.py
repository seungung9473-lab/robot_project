"""User-confirmed QR expiry format; deterministic calendar/rank routing."""
from datetime import date,datetime
from zoneinfo import ZoneInfo
import re,math

# 2026-11-10 사용자 결정: 유통기한 당일 것도 폐기한다(기한 당일 = 만료). 이전에는 당일까지 유효로 보았다.
DISCARD_ON_EXPIRY_DATE=True
def is_expired(expiry,today):
 return expiry<=today if DISCARD_ON_EXPIRY_DATE else expiry<today

def today_seoul():return datetime.now(ZoneInfo('Asia/Seoul')).date()

def parse_qr_expiry(text):
 # User confirmed one item-type letter followed by YYMMDD, e.g. O261101.
 if not isinstance(text,str):raise ValueError('QR text required')
 m=re.fullmatch(r'([A-Za-z])(\d{2})(\d{2})(\d{2})',text.strip())
 if not m:raise ValueError('Expected type letter + YYMMDD')
 kind,yy,mm,dd=m.groups();return {'item_type':kind,'expiry_date':date(2000+int(yy),int(mm),int(dd)).isoformat(),'qr_text':text.strip()}

# 인쇄 날짜 합의: 일치 판독이 충분히 많고 어긋난 판독이 단 1건이며, 그 1건이 만료/미만료 판정을 바꾸지 않을 때만 소수 오독으로 본다.
MIN_MAJORITY_SOURCES=4;MAX_OUTLIER_SOURCES=1

def printed_consensus(observations,today=None):
 today=today or today_seoul();by_value={};value_of={}
 for item in observations:
  if not isinstance(item,dict) or item.get('source')!='printed_numeric':continue
  confidence=item.get('confidence',0)
  if type(confidence) not in (int,float) or not math.isfinite(confidence) or not .9<=confidence<=1:continue
  text=item.get('date_text','');source=item.get('evidence_id')
  if not isinstance(text,str) or not isinstance(source,str):continue
  full=re.fullmatch(r'(\d{4})[-./](\d{1,2})[-./](\d{1,2})',text.strip());short=re.fullmatch(r'(\d{1,2})[./-](\d{1,2})',text.strip())
  try:
   if full:value=date(*map(int,full.groups()))
   elif short:value=date(today.year,*map(int,short.groups()))
   else:continue
  except ValueError:continue
  if source in value_of and value_of[source]!=value:return None  # 같은 증거에서 서로 다른 날짜는 충돌
  value_of[source]=value;by_value.setdefault(value,set()).add(source)
 if not by_value:return None
 ranked=sorted(by_value.items(),key=lambda kv:-len(kv[1]));top,top_sources=ranked[0]
 if len(top_sources)<2:return None
 others=ranked[1:]
 if not others:return top
 if len(top_sources)<MIN_MAJORITY_SOURCES:return None
 if len(others)>1 or len(others[0][1])>MAX_OUTLIER_SOURCES:return None
 if is_expired(others[0][0],today)!=is_expired(top,today):return None  # 폐기 여부가 갈리는 충돌은 절대 무시하지 않는다
 return top

def classify(observations,today=None,**unused):
 today=today or today_seoul();numeric=printed_consensus(observations,today);dates=[];seen=set()
 if numeric is not None:
  expired=is_expired(numeric,today)
  return {'status':'expired' if expired else 'not_expired','expiry_date':numeric.isoformat(),'today':today.isoformat(),'remaining_days':(numeric-today).days,'destination':'trash_drop' if expired else None,'destination_pad_id':None,'date_source':'printed_numeric','rule':'Printed MM.DD uses execution year; expired on and after the expiry date (discard on the day)'}
 for item in observations:
  if not isinstance(item,dict):continue
  source=item.get('evidence_id')
  if not isinstance(source,str):continue
  try:parsed=parse_qr_expiry(item['qr_text'])
  except (ValueError,TypeError,KeyError):continue
  value=date.fromisoformat(parsed['expiry_date'])
  # Even a second, conflicting QR within the same image blocks disposal.
  if dates and value!=dates[0]:return {'status':'unknown','reason':'Conflicting QR expiry dates; do not move','today':today.isoformat()}
  if source in seen:continue
  seen.add(source);dates.append(value)
 if len(dates)<2:return {'status':'unknown','reason':'Require matching QR reads from two distinct observations','today':today.isoformat()}
 if len(set(dates))!=1:return {'status':'unknown','reason':'Conflicting QR expiry dates; do not move','today':today.isoformat()}
 expiry=dates[0];expired=is_expired(expiry,today)
 return {'status':'expired' if expired else 'not_expired','expiry_date':expiry.isoformat(),'today':today.isoformat(),'remaining_days':(expiry-today).days,'destination':'trash_drop' if expired else None,'destination_pad_id':None,'date_source':'qr_fallback','rule':'Expired on and after the expiry date (discard on the day); QR year = 2000 + YY'}

def rank_destinations(decisions,reference_date=None,pad_ids=None):
 # pad_ids={'discount':id,'regular':id} comes from the 할인/정규 stickers read on the pads; default keeps the fixed ID5/ID6 layout.
 pad_ids=pad_ids or {'discount':5,'regular':6}
 if pad_ids['discount']==pad_ids['regular'] or {pad_ids['discount'],pad_ids['regular']}!={5,6}:raise ValueError('Pad labels must map to the two calibrated pads')
 reference_date=reference_date or today_seoul();valid=[o for o in decisions if o['status']=='not_expired']
 # The user specified comparing TWO non-expired dates, not a fixed-day threshold.
 distinct={o['expiry_date'] for o in valid}
 if len(distinct)>2:raise ValueError('More than two non-expired dates: define a grouping rule before moving')
 distances={key:abs((date.fromisoformat(key)-reference_date).days) for key in distinct}
 nearest=min(distances.values()) if distances else None
 for o in valid:
  gap=distances[o['expiry_date']];pid=pad_ids['discount'] if gap==nearest else pad_ids['regular'];o.update(destination='pad%d'%pid,destination_pad_id=pid,pad_label='할인' if gap==nearest else '정규',reference_date=reference_date.isoformat(),distance_to_reference_days=gap)
 # Equal distances (including equal dates) share ID5; no arbitrary far label.
 return decisions

def validate_route(route,today=None):
 decision=route.get('expiry_decision')
 if not decision:return
 today=today or today_seoul()
 if decision.get('status')=='unknown':raise ValueError('Date uncertain; execution blocked')
 expiry=date.fromisoformat(decision['expiry_date']);trash=route.get('destination')=='trash_drop'
 if trash!=is_expired(expiry,today):raise ValueError('Expiry state changed; classify and plan again')
 if decision.get('today')!=today.isoformat():raise ValueError('Calendar day changed; classify and plan again')
 if not trash and route.get('destination_pad_id') not in [5,6]:raise ValueError('No ranked destination')
