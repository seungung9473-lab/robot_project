"""Initialize SAM2 from depth foreground anywhere above the calibrated disc."""
from pathlib import Path
import cv2,json,numpy as np,argparse
from placement_geometry import load_context,depth_table,object_region

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--session',type=Path,required=True);args=parser.parse_args();session=args.session;context=load_context(session);out={}
    for view in json.loads((session/'session.json').read_text())['views']:
        folder=session/view['path'];meta=json.loads((folder/'metadata.json').read_text())
        for camera in ['cam1','cam2']:
            data=meta['cameras'][camera];raw=cv2.imread(str(folder/data['depth_file']),-1) if data['depth_file'].endswith('.png') else np.load(folder/data['depth_file']);image=cv2.imread(str(folder/data['color_file']));T=context['calibrations'][camera][f'T_table_{camera}_color_optical'];points,valid=depth_table(raw,data['depth_scale_m'],data['camera_info'],T);seed=(valid&object_region(points,context)).astype('uint8')
            connected=cv2.morphologyEx(seed,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8));count,labels,stats,centers=cv2.connectedComponentsWithStats(connected);indices=sorted([i for i in range(1,count) if stats[i,4]>100],key=lambda i:stats[i,4],reverse=True)
            # Expanded observation region can contain the parked robot tool. Keep
            # components whose measured XY centre is actually above the support.
            component_centers={i:np.median(points[(labels==i)&(seed>0)],axis=0) for i in indices if np.any((labels==i)&(seed>0))}
            axis_xy=np.asarray(context['axis_point_table_m'])[:2]
            indices=[i for i in indices if i in component_centers and np.linalg.norm(component_centers[i][:2]-axis_xy)<context['support_radius_m']]
            if not indices:raise RuntimeError(f'No supported depth foreground above disc: {folder} {camera}')
            if len(indices)>1 and stats[indices[1],4]>.25*stats[indices[0],4]:raise RuntimeError('Multiple foreground objects: select one target before scanning')
            i=indices[0];component=(labels==i)&(seed>0);x,y,w,h,area=stats[i];height,width=raw.shape;box=[max(0,int(x)-5),max(0,int(y)-5),min(width-1,int(x+w)+5),min(height-1,int(y+h)+5)];distance=cv2.distanceTransform(component.astype('uint8'),cv2.DIST_L2,3);yy,xx=np.unravel_index(np.argmax(distance),distance.shape)
            if distance[yy,xx]<2:raise RuntimeError('Foreground too thin for reliable SAM2 initialization')
            out[view['path']+'/'+camera]={'image':str((folder/data['color_file']).resolve()),'box_xyxy':box,'positive_xy':[int(xx),int(yy)],'prompt_source':'depth foreground above calibrated disc; no color or axis-centred-object assumption','seed_area':int(component.sum()),'seed_center_table_m':np.median(points[component],axis=0).tolist()}
    dest=session/'sam2';dest.mkdir(exist_ok=True);(dest/'prompts.json').write_text(json.dumps(out,indent=2)+'\n');print('Depth foreground prompts',len(out),flush=True)
if __name__=='__main__':main()
