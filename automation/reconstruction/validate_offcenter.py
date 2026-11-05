"""Offline regression for measured-axis alignment and off-centre suction selection."""
from pathlib import Path
import json,subprocess,sys
import numpy as np
from scipy.spatial.transform import Rotation
from reconstruct import unrotate
from placement_geometry import contact_target,object_region,load_context
r=Path(__file__).resolve().parent;out=r/'validation_offcenter';out.mkdir(exist_ok=True);rng=np.random.default_rng(20261101);axis=np.array([1.095,.235,.059]);center=axis+np.array([.030,.015,.030]);grid=np.linspace(-.025,.025,51);xx,yy=np.meshgrid(grid,grid);top=np.column_stack([xx.ravel()+center[0],yy.ravel()+center[1],np.full(xx.size,.119)]);parts=[top]
for dimension in [0,1]:
 for sign in [-1,1]:
  uu,zz=np.meshgrid(grid,np.linspace(.066,.119,50));p=np.zeros((uu.size,3));p[:,dimension]=center[dimension]+sign*.025;p[:,1-dimension]=center[1-dimension]+uu.ravel();p[:,2]=zz.ravel();parts.append(p)
reference=np.vstack(parts);aligned=[]
for angle in range(0,-360,-30):
 R=Rotation.from_euler('z',angle,degrees=True).as_matrix();observed=(reference-axis)@R.T+axis;recovered=unrotate(observed,angle,axis,1)
 assert np.max(np.abs(recovered-reference))<1e-12
 aligned.append(unrotate(observed+rng.normal(0,.0001,observed.shape),angle,axis,1))
# Test centroid trajectory independently: an off-centre centre traces a circle and must not be collapsed to the axis.
recovered_center=unrotate(((center-axis)@Rotation.from_euler('z',-90,degrees=True).as_matrix().T+axis)[None,:],-90,axis,1)[0];assert np.linalg.norm(recovered_center-center)<1e-12
p=np.vstack(aligned);assert np.linalg.norm(np.median(p[:,:2],axis=0)-center[:2])<.001
for c in ['cam1','cam2']:np.savez_compressed(out/(c+'_object.npz'),points=p,colors=np.tile([220,50,30],(len(p),1)))
subprocess.run([sys.executable,str(r/'find_suction_surface.py'),'--folder',str(out)],check=True,stdout=subprocess.DEVNULL)
d=json.loads((out/'suction_candidates.json').read_text());candidate=d['selected_candidate'];point=np.array(candidate['nominal_contact_position_table_m']);assert d['results'][candidate['camera']]['geometry_pass'];assert np.linalg.norm(point[:2]-center[:2])<.002;assert np.linalg.norm(point[:2]-axis[:2])>.025
contact=contact_target(point,-.002);assert np.allclose(contact[:2],point[:2]);assert np.isclose(contact[2],point[2]-.002)
# Explicit nonzero scan reference: axes do not move when the first camera view changes.
ref=37.;angle=-83.;R=Rotation.from_euler('z',angle,degrees=True).as_matrix();observed=(reference-axis)@R.T+axis;registered=unrotate(observed,angle-ref,axis,1);expected=(reference-axis)@Rotation.from_euler('z',ref,degrees=True).as_matrix().T+axis;assert np.max(np.abs(registered-expected))<1e-12
result={'rotation_roundtrip_pass':True,'nonzero_reference_pass':True,'object_offset_from_axis_mm':[30,15],'expected_center_xy_m':center[:2].tolist(),'candidate_xy_m':point[:2].tolist(),'candidate_xy_error_mm':float(np.linalg.norm(point[:2]-center[:2])*1000),'candidate_does_not_snap_to_axis':True,'contact_uses_candidate_xy':True,'geometry_pass':True,'hardware_moved':False,'limitations':'synthetic geometry regression; camera/robot physical accuracy not certified'};(out/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
