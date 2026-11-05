"""Decode QR expiry per SAM2 instance across stopped views. No network/ROS/motion."""
import argparse,hashlib,json,sys,time
from pathlib import Path
import cv2,numpy as np
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R/'vendor/qr_decoder'))
import zxingcpp
from expiry_policy import parse_qr_expiry

def read_qr_dates(session,only_objects=None):
 s=Path(session).resolve();manifest=json.loads((s/'session.json').read_text())
 if manifest['status']!='complete' or time.time()-(s/'session.json').stat().st_mtime>1800:raise ValueError('Fresh completed scan required')
 objects=[o for o in sorted((s/'objects').glob('object_*')) if only_objects is None or o.name in only_objects];records={o.name:[] for o in objects};unassigned=[]
 if not objects:raise ValueError('AI identities and per-object SAM2 masks required')
 for view in manifest['views']:
  folder=s/view['path'];meta=json.loads((folder/'metadata.json').read_text())
  for cam in ['cam1','cam2']:
   image=folder/meta['cameras'][cam]['color_file'];im=cv2.imread(str(image));masks={o.name:cv2.imread(str(o/'sam2'/view['path']/(cam+'_mask.png')),0) for o in objects}
   if im is None or any(m is None or m.shape!=im.shape[:2] for m in masks.values()):raise ValueError('Missing/mismatched RGB or instance mask')
   decoded=[]
   def decode(image,offset=(0,0),scale=1):
    for code in zxingcpp.read_barcodes(image,formats=zxingcpp.BarcodeFormat.QRCode,try_rotate=True,try_downscale=True,try_invert=True):
     corners=[code.position.top_left,code.position.top_right,code.position.bottom_right,code.position.bottom_left];poly=np.array([[round(p.x/scale+offset[0]),round(p.y/scale+offset[1])] for p in corners],np.int32)
     if not any(text==code.text and np.linalg.norm(old.mean(0)-poly.mean(0))<5 for text,old in decoded):decoded.append((code.text,poly))
   decode(im)
   # A QR can be too small for full-frame detection; interpolation does not
   # invent payloads. Accept only the decoder's valid QR/checksum result.
   for mask in masks.values():
    yy,xx=np.nonzero(mask>127)
    if len(xx)<100:continue
    x0,y0=max(0,int(xx.min())-10),max(0,int(yy.min())-10);x1,y1=min(im.shape[1],int(xx.max())+11),min(im.shape[0],int(yy.max())+11);roi=im[y0:y1,x0:x1]
    for scale in [2,4]:decode(cv2.resize(roi,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC),(x0,y0),scale)
   for text,poly in decoded:
    try:date_info=parse_qr_expiry(text)
    except ValueError:continue
    region=np.zeros(im.shape[:2],np.uint8);cv2.fillConvexPoly(region,poly,1);area=int(region.sum())
    if area<9:continue
    support={key:float(np.count_nonzero((mask>127)&(region>0))/area) for key,mask in masks.items()};assigned=[key for key,value in support.items() if value>=.65]
    if len(assigned)!=1:unassigned.append({'view':view['path'],'camera':cam,'qr_text':text,'mask_support':support});continue
    records[assigned[0]].append({'evidence_id':view['path']+'/'+cam,'qr_text':text,**date_info,'polygon_xy':poly.tolist(),'mask_support':support[assigned[0]],'source_image_sha256':hashlib.sha256(image.read_bytes()).hexdigest()})
 report={'source_scan':str(s),'read_at':time.time(),'reader':'ZXing QR + exact user-confirmed type-letter/YYMMDD format','objects':[{'object':key,'observations':value} for key,value in records.items()],'unassigned_qr':unassigned,'robot_commands_sent':False};(s/'qr_fallback_observations.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));return report
def read_dates(session,use_numeric=True):
 s=Path(session).resolve()
 if use_numeric:
  from read_numeric_dates import read_numeric_dates
  from expiry_policy import printed_consensus
  report=read_numeric_dates(s);missing={o['object'] for o in report['objects'] if printed_consensus(o['observations']) is None}
  if missing:
   qr=read_qr_dates(s,missing);by_key={o['object']:o for o in qr['objects']}
   for obj in report['objects']:
    if obj['object'] in by_key:obj['observations']+=by_key[obj['object']]['observations']
   report['qr_fallback_objects']=sorted(missing);report['unassigned_qr']=qr['unassigned_qr']
 else:report=read_qr_dates(s)
 (s/'expiry_observations.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));return report

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('session',type=Path);a=p.parse_args();print(json.dumps(read_dates(a.session),ensure_ascii=False,indent=2))
