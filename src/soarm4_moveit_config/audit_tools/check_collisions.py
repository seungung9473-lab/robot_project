#!/usr/bin/env python3
"""Offline FCL triangle-mesh collision audit. Never communicates with motors."""
from pathlib import Path
import sys, json, time, hashlib, itertools, argparse, xml.etree.ElementTree as ET
import numpy as np
import fcl,trimesh
ROOT=Path(__file__).resolve().parents[1]
SNAPSHOT=Path(__file__).resolve().parent/'model_snapshot'
DEFAULT_PACKAGE=SNAPSHOT if SNAPSHOT.is_dir() else Path.home()/'robot_ws/src/soarm4_description'

class Auditor:
    def __init__(self,package=DEFAULT_PACKAGE):
        self.package=Path(package).resolve();sys.path.insert(0,str(self.package/'tools'))
        from validate_geometry import fk
        from mesh_geometry import read_stl
        self.fk=fk;self.read_stl=read_stl;self.robot=ET.parse(self.package/'urdf/soarm4.urdf').getroot();self.model=json.loads((self.package/'config/model.json').read_text());self.joints=self.model['joints'];self.names=[j['name'] for j in self.joints]
        self.limits=np.array([[float(j['limit'][v]) for v in ['lower','upper']] for j in self.joints]);self.objects=[];self.meshes=[];self.quality=[]
        for entry in self.model['visuals']:
            link=self.robot.find("link[@name='"+entry['name']+"_visual_link']");collision=link.find('collision');assert collision is not None
            mesh_element=collision.find('geometry/mesh');assert mesh_element is not None
            origin=collision.find('origin');assert origin.get('xyz')=='0 0 0' and origin.get('rpy')=='0 0 0';assert mesh_element.get('scale','1 1 1')=='1 1 1'
            path=self.package/mesh_element.get('filename').split('package://soarm4_description/')[1];tri=self.read_stl(path)
            mesh=trimesh.Trimesh(vertices=tri.reshape(-1,3),faces=np.arange(tri.size//3).reshape(-1,3),process=True)
            v=np.asarray(mesh.vertices,dtype=np.float64);faces=np.asarray(mesh.faces,dtype=np.int32);geom=fcl.BVHModel();geom.beginModel(len(v),len(faces));geom.addSubModel(v,faces);geom.endModel();obj=fcl.CollisionObject(geom)
            group=entry['link'];rigid_group='fixed_base' if group in ['base_link','electronics_base_link'] else group
            self.meshes.append({'name':entry['name'],'link':entry['name']+'_visual_link','owner':group,'rigid_group':rigid_group,'mesh':mesh,'geom':geom,'path':str(path),'color':entry['color']});self.objects.append(obj)
            self.quality.append({'name':entry['name'],'vertices':len(v),'triangles':len(faces),'watertight':bool(mesh.is_watertight),'winding_consistent':bool(mesh.is_winding_consistent),'degenerate_triangles':int(np.sum(mesh.area_faces<1e-16)),'bounds_m':mesh.bounds.tolist(),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
        self.pairs=list(itertools.combinations(range(len(self.objects)),2));self.dynamic_pairs=[(i,j) for i,j in self.pairs if self.meshes[i]['rigid_group']!=self.meshes[j]['rigid_group']];self.static_pairs=[p for p in self.pairs if p not in self.dynamic_pairs]
        self.adjacent={frozenset([j['parent'] if j['parent']!='base_link' else 'fixed_base',j['child']]) for j in self.joints}
        self.last_frames=None;self.last_q=None;self.last_bounds=None
        self.boolean_request=fcl.CollisionRequest(num_max_contacts=1,enable_contact=False)
    def set_pose(self,q):
        q=np.array(q,dtype=float);T=self.fk(self.robot,dict(zip(self.names,q)));bounds=[]
        for obj,m in zip(self.objects,self.meshes):
            F=T[m['link']];obj.setTransform(fcl.Transform(np.ascontiguousarray(F[:3,:3]),np.ascontiguousarray(F[:3,3])))
            corners=trimesh.bounds.corners(m['mesh'].bounds)@F[:3,:3].T+F[:3,3];bounds.append([corners.min(0),corners.max(0)])
        self.last_frames=T;self.last_q=q;self.last_bounds=np.array(bounds)
    def inside_limits(self,q):return np.logical_and(np.array(q)>=self.limits[:,0],np.array(q)<=self.limits[:,1])
    def collision_pairs(self,include_static=False):
        out=[]
        for i,j in self.pairs if include_static else self.dynamic_pairs:
            b1,b2=self.last_bounds[i],self.last_bounds[j]
            if np.any(b1[1]<b2[0]-1e-10) or np.any(b2[1]<b1[0]-1e-10):continue
            result=fcl.CollisionResult();fcl.collide(self.objects[i],self.objects[j],self.boolean_request,result)
            if result.is_collision:out.append((i,j))
        return out
    def details(self,i,j,max_contacts=256,nearby=True):
        a,b=self.meshes[i],self.meshes[j];req=fcl.CollisionRequest(num_max_contacts=max_contacts,enable_contact=True);result=fcl.CollisionResult();fcl.collide(self.objects[i],self.objects[j],req,result)
        item={'mesh1':a['name'],'mesh2':b['name'],'owner1':a['owner'],'owner2':b['owner'],'same_rigid_group':a['rigid_group']==b['rigid_group'],'adjacent_kinematic_groups':frozenset([a['rigid_group'],b['rigid_group']]) in self.adjacent,'surface_intersection':bool(result.is_collision),'contacts_returned':len(result.contacts),'contact_cap':max_contacts}
        if result.contacts:
            positions=np.array([c.pos for c in result.contacts]);item['contact_points_m']=positions.tolist();item['contact_bounds_m']=[positions.min(0).tolist(),positions.max(0).tolist()];item['triangle_pair_ids']=[[int(c.b1),int(c.b2)] for c in result.contacts];item['fcl_reported_depths_m']=[float(c.penetration_depth) for c in result.contacts]
        elif nearby:
            req=fcl.DistanceRequest(enable_nearest_points=True);res=fcl.DistanceResult();distance=fcl.distance(self.objects[i],self.objects[j],req,res);item['surface_distance_m']=float(distance);item['nearest_points_m']=[np.asarray(x).tolist() for x in res.nearest_points]
        return item
    def snapshot(self,q,details=True,include_static=True):
        self.set_pose(q);pairs=self.collision_pairs(include_static);limit_mask=self.inside_limits(q)
        report={'angles_rad':list(map(float,q)),'angles_deg':np.degrees(q).tolist(),'within_canonical_limits':{n:bool(ok) for n,ok in zip(self.names,limit_mask)},'dynamic_intersections':[],'fixed_assembly_intersections':[]}
        for i,j in pairs:
            d=self.details(i,j) if details else {'mesh1':self.meshes[i]['name'],'mesh2':self.meshes[j]['name']}
            key='fixed_assembly_intersections' if self.meshes[i]['rigid_group']==self.meshes[j]['rigid_group'] else 'dynamic_intersections';report[key].append(d)
        return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--package',default=str(DEFAULT_PACKAGE));parser.add_argument('--out',default=str(Path(__file__).parent));args=parser.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    start=time.monotonic();a=Auditor(args.package);print('Built',len(a.meshes),'mesh BVHs; dynamic pairs',len(a.dynamic_pairs),'in',time.monotonic()-start,'s',flush=True)
    (out/'mesh_quality.json').write_text(json.dumps(a.quality,indent=2))
    poses={name:a.snapshot(a.model['poses'][name]) for name in ['home','zero','rest']}
    (out/'reference_poses_raw.json').write_text(json.dumps(poses,indent=2))
    for name,r in poses.items():print(name,'limits',r['within_canonical_limits'],'dynamic',[(p['mesh1'],p['mesh2'],p.get('contacts_returned')) for p in r['dynamic_intersections']],'fixed',len(r['fixed_assembly_intersections']),flush=True)
    rng=np.random.default_rng(9473);t=time.monotonic();n=100
    for q in rng.uniform(a.limits[:,0],a.limits[:,1],size=(n,4)):a.set_pose(q);a.collision_pairs()
    print('Benchmark',n,'states',time.monotonic()-t,'s',flush=True)
