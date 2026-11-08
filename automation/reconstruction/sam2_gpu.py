"""GPU SAM2 inference with shared image features and batched independent prompts."""
import json,time,hashlib,contextlib
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
import torch

def run_gpu(sessions,predictor,batch_size=2,write_review=True):
    groups={};states=[];start=time.monotonic()
    for session in sessions:
        dest=Path(session)/'sam2';prompts=json.loads((dest/'prompts.json').read_text());state={'dest':dest,'results':[]};states.append(state)
        for key,p in prompts.items():groups.setdefault(str(Path(p['image']).resolve()),[]).append((state,key,p))
    images=list(groups);encodes=0
    for offset in range(0,len(images),batch_size):
        chunk=images[offset:offset+batch_size];active=[path for path in chunk if any(not p.get('occluded',False) for _,_,p in groups[path])]
        decoded={path:np.array(Image.open(path).convert('RGB')) for path in chunk}
        if active:
            predictor.set_image_batch([decoded[path] for path in active]);encodes+=len(active)
            selected={}
            for image_index,path in enumerate(active):
                by_length={}
                for item in groups[path]:
                    if not item[2].get('occluded',False):by_length.setdefault(1+len(item[2].get('negative_xy',[])),[]).append(item)
                for _,items in by_length.items():
                    coords=np.array([[p['positive_xy']]+p.get('negative_xy',[]) for _,_,p in items],dtype=np.float32)
                    labels=np.array([[1]+[0]*len(p.get('negative_xy',[])) for _,_,p in items],dtype=np.int32)
                    boxes=np.array([p['box_xyxy'] for _,_,p in items])
                    mask_input,coords,labels,boxes=predictor._prep_prompts(coords,labels,boxes,None,True,img_idx=image_index)
                    ms,sc,_=predictor._predict(coords,labels,boxes,mask_input,True,img_idx=image_index)
                    ms=ms.float().detach().cpu().numpy();sc=sc.float().detach().cpu().numpy()
                    for item,mm,ss in zip(items,ms,sc):selected[(path,str(item[0]['dest']),item[1])]=(mm[int(ss.argmax())].astype(bool),float(ss.max()))
        for path in chunk:
            rgb=decoded[path]
            for state,key,p in groups[path]:
                view,camera=key.split('/');out=state['dest']/view;out.mkdir(parents=True,exist_ok=True)
                if p.get('occluded',False):mask=np.zeros(rgb.shape[:2],bool);score=None
                else:
                    mask,score=selected[(path,str(state['dest']),key)];area=int(mask.sum());h,w=mask.shape
                    if not 100<area<h*w*.12:raise RuntimeError('Mask area check failed '+key)
                Image.fromarray(mask.astype('uint8')*255).save(out/(camera+'_mask.png'))
                if write_review:
                    overlay=rgb.copy();overlay[mask]=(rgb[mask]*.55+np.array([0,255,80])*.45).astype('uint8');im=Image.fromarray(overlay)
                    if 'box_xyxy' in p:ImageDraw.Draw(im).rectangle(p['box_xyxy'],outline='yellow',width=1)
                    im.save(out/(camera+'_overlay.jpg'))
                state['results'].append({'view':view,'camera':camera,'observed':not p.get('occluded',False),'score':score,'pixels':int(mask.sum()),'mask_file':str(out/(camera+'_mask.png'))})
    torch.cuda.synchronize()
    stats={'sessions':len(states),'unique_images':len(groups),'image_encodes':encodes,'batch_size':batch_size,'seconds':time.monotonic()-start,'write_review':write_review}
    for state in states:
        payload={'model':'SAM2.1 Hiera Tiny','device':'cuda','shared_features':stats,'results':state['results']};(state['dest']/'manifest.json').write_text(json.dumps(payload,indent=2));(state['dest']/'inference_progress.json').write_text(json.dumps(state['results'],indent=2))
    return stats
