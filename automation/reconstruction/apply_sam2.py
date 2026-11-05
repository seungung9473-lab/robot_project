"""Offline SAM2.1 image inference for captured turntable views. No ROS/hardware."""
import json,hashlib,time,os
from pathlib import Path
import numpy as np,torch
from PIL import Image,ImageDraw
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
import argparse
a=argparse.ArgumentParser();a.add_argument('--session',type=Path);args=a.parse_args()
r=Path(__file__).resolve().parent;session=args.session or r/'multiview/20261101_194509_024461';dest=session/'sam2';prompts=json.loads((dest/'prompts.json').read_text());torch.set_num_threads(min(6,os.cpu_count() or 2));checkpoint=r/'checkpoints/sam2.1_hiera_tiny.pt';predictor=SAM2ImagePredictor(build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml',str(checkpoint),device='cpu',apply_postprocessing=False));results=[];tiles=[]
with torch.inference_mode():
 for key,p in prompts.items():
  t=time.monotonic();rgb=np.array(Image.open(p['image']).convert('RGB'));view,c=key.split('/');out=dest/view;out.mkdir(exist_ok=True)
  if p.get('occluded',False):
   Image.fromarray(np.zeros(rgb.shape[:2],dtype=np.uint8)).save(out/(c+'_mask.png'));results.append({'view':view,'camera':c,'observed':False,'reason':'no visible target seed; not hallucinated'});print(key,'occluded',flush=True);continue
  predictor.set_image(rgb);negative=p.get('negative_xy',[]);coords=np.array([p['positive_xy']]+negative);labels=np.array([1]+[0]*len(negative));masks,scores,_=predictor.predict(point_coords=coords,point_labels=labels,box=np.array(p['box_xyxy']),multimask_output=True)
  mask=masks[int(np.argmax(scores))].astype(bool);score=float(max(scores));area=int(mask.sum());h,w=mask.shape
  if not 100<area<h*w*.12:raise RuntimeError('Mask area check failed '+key)
  view,c=key.split('/');out=dest/view;out.mkdir(exist_ok=True);Image.fromarray((mask*255).astype(np.uint8)).save(out/(c+'_mask.png'));overlay=rgb.copy();overlay[mask]=(rgb[mask]*.55+np.array([0,255,80])*.45).astype(np.uint8);im=Image.fromarray(overlay);draw=ImageDraw.Draw(im);draw.rectangle(p['box_xyxy'],outline='yellow',width=1);draw.text((8,8),key+' SAM2 %.3f'%score,fill='white');im.save(out/(c+'_overlay.jpg'));im.thumbnail((320,240));tiles.append(im)
  item={'view':view,'camera':c,'mask_file':str(out/(c+'_mask.png')),'score':score,'pixels':area,'seconds':time.monotonic()-t};results.append(item);(dest/'inference_progress.json').write_text(json.dumps(results,indent=2)+'\n');print(key,'score',round(score,3),'pixels',area,'sec',round(item['seconds'],2),flush=True)
contact=Image.new('RGB',(320*4,240*6),'white')
for i,im in enumerate(tiles):contact.paste(im,((i%4)*320,(i//4)*240))
contact.save(dest/'mask_review.jpg');(dest/'manifest.json').write_text(json.dumps({'model':'SAM2.1 Hiera Tiny','device':'CPU','checkpoint_sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),'inference':'independent image prompts; not cross-camera video tracking','prompt_method':'depth foreground on calibrated turntable; masks are SAM2 predictions','results':results},indent=2)+'\n');print('COMPLETE',dest,flush=True)
