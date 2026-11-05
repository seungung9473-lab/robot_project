"""Joint, non-stacking pad reservations. Pure geometry; never moves hardware."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

def footprint(points):
    p=np.asarray(points,float)[:,:2]
    if not np.isfinite(p).all():raise ValueError('Nonfinite footprint')
    return p[ConvexHull(p).vertices]

def separated(a,b,gap=.002):
    for poly in (a,b):
        edges=np.roll(poly,-1,axis=0)-poly
        axes=np.column_stack([-edges[:,1],edges[:,0]])
        axes/=np.linalg.norm(axes,axis=1)[:,None]
        pa=a@axes.T;pb=b@axes.T
        if np.any((pa.max(0)+gap<=pb.min(0))|(pb.max(0)+gap<=pa.min(0))):return True
    return False

def reserve(objects,center,radius,edge=.002,gap=.002,step=.002):
    """Search all translations jointly before committing the first placement.

    Footprints must already have the intended placement yaw. No scale reduction,
    stacking or rotation is assumed. A failed discrete search is not proof of
    mathematical impossibility.
    """
    center=np.asarray(center,float);polys={k:footprint(v) for k,v in objects.items()}
    offsets=np.arange(-radius,radius+step/2,step)
    grid=np.array([(x,y) for x in offsets for y in offsets])
    grid=grid[np.argsort(np.linalg.norm(grid,axis=1),kind='stable')]
    options={k:[p+t for t in grid if np.max(np.linalg.norm(p+t,axis=1))<=radius-edge+1e-9] for k,p in polys.items()}
    order=sorted(polys,key=lambda k:len(options[k]));chosen={}
    def search(i):
        if i==len(order):return True
        k=order[i]
        for p in options[k]:
            if all(separated(p,q,gap) for q in chosen.values()):
                chosen[k]=p
                if search(i+1):return True
                del chosen[k]
        return False
    if not search(0):raise ValueError('No joint placement found; keep objects and request a revised layout')
    return {'version':1,'stacking':False,'pad_center_table_m':center.tolist(),'radius_m':radius,'edge_margin_m':edge,'object_gap_m':gap,'placements':{k:{'offset_xy_m':(chosen[k][0]-polys[k][0]).tolist(),'reserved_polygon_table_m':(chosen[k]+center[:2]).tolist()} for k in polys}}

def validate_actual(plan,key,points):
    p=footprint(points);center=np.array(plan['pad_center_table_m'])[:2]
    if np.max(np.linalg.norm(p-center,axis=1))>plan['radius_m']-plan['edge_margin_m']+1e-8:raise ValueError('Actual placement exceeds pad edge')
    for other,item in plan['placements'].items():
        if other!=key and not separated(p,np.array(item['reserved_polygon_table_m']),plan['object_gap_m']):raise ValueError('Actual placement overlaps another reserved position')
    for other,item in plan.get('completed',{}).items():
        if other!=key and not separated(p,footprint(item['points_table_m']),plan['object_gap_m']):raise ValueError('Actual placement overlaps an already placed object')
    return p

if __name__=='__main__':
    ap=argparse.ArgumentParser(description='Reserve all objects on one pad before the first pick')
    ap.add_argument('spec',type=Path);ap.add_argument('output',type=Path);a=ap.parse_args();s=json.loads(a.spec.read_text());objects={}
    for k,item in s['objects'].items():
        pts=np.load(item['cloud_file'])['points'];origin=np.asarray(item['placement_origin_table_m']);R=np.asarray(item.get('placement_rotation',np.eye(3)))
        objects[k]=(pts-origin)@R.T
    result=reserve(objects,s['pad_center_table_m'],s['diameter_m']/2);result['pad_id']=s['pad_id'];result['completed']={};a.output.write_text(json.dumps(result,indent=2)+'\n')
