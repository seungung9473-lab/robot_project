"""Share image features across independent object prompts; no model/hardware IO."""
import json,time
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw

def run_sessions(sessions,predictor,checkpoint_sha256):
    groups={};states=[]
    for session in sessions:
        dest=Path(session)/'sam2';prompts=json.loads((dest/'prompts.json').read_text())
        state={'dest':dest,'results':[],'tiles':[]};states.append(state)
        for key,prompt in prompts.items():
            groups.setdefault(str(Path(prompt['image']).resolve()),[]).append((state,key,prompt))
    encodes=0
    for image_path,items in groups.items():
        rgb=np.array(Image.open(image_path).convert('RGB'))
        if any(not p.get('occluded',False) for _,_,p in items):
            predictor.set_image(rgb);encodes+=1
        for state,key,p in items:
            start=time.monotonic();view,camera=key.split('/');out=state['dest']/view;out.mkdir(parents=True,exist_ok=True)
            if p.get('occluded',False):
                Image.fromarray(np.zeros(rgb.shape[:2],dtype=np.uint8)).save(out/(camera+'_mask.png'))
                state['results'].append({'view':view,'camera':camera,'observed':False,'reason':'no visible target seed; not hallucinated'})
                continue
            negative=p.get('negative_xy',[]);coords=np.array([p['positive_xy']]+negative);labels=np.array([1]+[0]*len(negative))
            masks,scores,_=predictor.predict(point_coords=coords,point_labels=labels,box=np.array(p['box_xyxy']),multimask_output=True)
            mask=masks[int(np.argmax(scores))].astype(bool);score=float(max(scores));area=int(mask.sum());h,w=mask.shape
            if not 100<area<h*w*.12:raise RuntimeError('Mask area check failed '+key)
            Image.fromarray((mask*255).astype(np.uint8)).save(out/(camera+'_mask.png'))
            overlay=rgb.copy();overlay[mask]=(rgb[mask]*.55+np.array([0,255,80])*.45).astype(np.uint8)
            im=Image.fromarray(overlay);draw=ImageDraw.Draw(im);draw.rectangle(p['box_xyxy'],outline='yellow',width=1);draw.text((8,8),key+' SAM2 %.3f'%score,fill='white');im.save(out/(camera+'_overlay.jpg'));im.thumbnail((320,240));state['tiles'].append(im)
            item={'view':view,'camera':camera,'mask_file':str(out/(camera+'_mask.png')),'score':score,'pixels':area,'seconds':time.monotonic()-start,'seconds_scope':'prompt decode and output; shared image encoding excluded'}
            state['results'].append(item);(state['dest']/'inference_progress.json').write_text(json.dumps(state['results'],indent=2)+'\n')
            print(state['dest'].parent.name,key,'score',round(score,3),'pixels',area,flush=True)
    stats={'sessions':len(states),'unique_images':len(groups),'image_encodes':encodes}
    for state in states:
        contact=Image.new('RGB',(320*4,240*6),'white')
        for i,im in enumerate(state['tiles']):contact.paste(im,((i%4)*320,(i//4)*240))
        dest=state['dest'];contact.save(dest/'mask_review.jpg')
        (dest/'manifest.json').write_text(json.dumps({'model':'SAM2.1 Hiera Tiny','device':'CPU','checkpoint_sha256':checkpoint_sha256,'inference':'independent image prompts; not cross-camera video tracking','prompt_method':'depth foreground on calibrated turntable; masks are SAM2 predictions','shared_features':stats,'results':state['results']},indent=2)+'\n')
        print('COMPLETE',dest,flush=True)
    return stats
