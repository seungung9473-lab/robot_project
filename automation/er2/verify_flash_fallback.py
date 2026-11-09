"""Simulate ER2 retirement, then test real Flash on an already cropped image."""
import datetime, json
from pathlib import Path
import cv2
from check_connection import load_key,request,APIError
from model_router import generate,policy
from select_object import build_payload,decode_selection
R=Path(__file__).resolve().parent

def main():
 latest=json.loads((R/'latest_selection.json').read_text());source=Path(latest['run']);meta=json.loads((source/'input_metadata.json').read_text());data=(source/'input.png').read_bytes();key,_=load_key()
 if not key:raise RuntimeError('No API key')
 payload=build_payload(meta,data)
 def validate(response):
  text=''.join(p.get('text','') for c in response.get('candidates',[]) for p in c.get('content',{}).get('parts',[]) if not p.get('thought',False));value=json.loads(text);width,height=meta['image_size_wh'];return decode_selection(value,width,height,meta['crop_xyxy'])
 def unavailable_primary(key,method,path,payload):
  if path.startswith(policy()['primary']+':'):raise APIError('Simulated ER2 model retirement; no primary network request',404,'NOT_FOUND')
  return request(key,method,path,payload)
 response,selection,routing=generate(key,payload,validate,transport=unavailable_primary,test_context='simulated_ER2_retirement_real_Flash_image_request')
 out=R/'runs'/('fallback_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True);(out/'api_response.json').write_text(json.dumps(response,ensure_ascii=False,indent=2));im=cv2.imread(str(source/'input.png'))
 if selection:
  x0,y0,x1,y1=selection['box_crop_xyxy'];x,y=selection['point_crop_xy'];cv2.rectangle(im,(x0,y0),(x1,y1),(0,220,0),2);cv2.drawMarker(im,(x,y),(0,0,255),cv2.MARKER_CROSS,14,2)
 cv2.imwrite(str(out/'selection_overlay.png'),cv2.resize(im,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST));result={'routing':routing,'selection':selection,'source_cropped_image':str(source/'input.png'),'response_model':response.get('modelVersion'),'usage_metadata':response.get('usageMetadata'),'robot_commands_sent':False,'simulation':'ER2 failure simulated locally; Flash inference is real','visual_verification_pending':True};(out/'fallback_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False,indent=2));print(out)
if __name__=='__main__':main()
