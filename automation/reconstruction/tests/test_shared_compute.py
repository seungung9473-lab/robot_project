import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch as mocked
import numpy as np,cv2
from PIL import Image
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from sam2_shared import run_sessions
from shared_observations import collect_plane_observations
import inclined_geometry

class Predictor:
 def __init__(self):self.encodes=0;self.calls=0
 def set_image(self,rgb):self.encodes+=1;self.shape=rgb.shape[:2]
 def predict(self,**kw):
  self.calls+=1;mask=np.zeros((1,*self.shape),bool);x,y=kw['point_coords'][0];mask[0,20:32,int(x):int(x)+12]=True
  return mask,np.array([.95]),None

class SharedComputeTests(unittest.TestCase):
 def test_same_image_features_shared_but_masks_remain_independent(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);image=root/'image.png';Image.fromarray(np.zeros((128,128,3),np.uint8)).save(image);sessions=[]
   for i in range(3):
    s=root/str(i);(s/'sam2').mkdir(parents=True);sessions.append(s)
    (s/'sam2/prompts.json').write_text(json.dumps({'view_001/cam1':{'image':str(image),'positive_xy':[10+20*i,25],'negative_xy':[],'box_xyxy':[0,0,127,127]}}))
   predictor=Predictor();stats=run_sessions(sessions,predictor,'test')
   self.assertEqual(predictor.encodes,1);self.assertEqual(predictor.calls,3);self.assertEqual(stats['image_encodes'],1)
   masks=[np.array(Image.open(s/'sam2/view_001/cam1_mask.png')) for s in sessions]
   self.assertFalse(np.array_equal(masks[0],masks[1]))

 def test_occluded_prompts_stay_empty_without_encoding(self):
  with tempfile.TemporaryDirectory() as folder:
   s=Path(folder);image=s/'image.png';Image.fromarray(np.zeros((128,128,3),np.uint8)).save(image);(s/'sam2').mkdir()
   (s/'sam2/prompts.json').write_text(json.dumps({'v/cam1':{'image':str(image),'occluded':True}}));p=Predictor();run_sessions([s],p,'test')
   self.assertEqual(p.encodes,0);self.assertEqual(p.calls,0);self.assertEqual(np.array(Image.open(s/'sam2/v/cam1_mask.png')).sum(),0)

 def test_different_source_images_never_share_features(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder);sessions=[]
   for i in range(2):
    s=root/str(i);(s/'sam2').mkdir(parents=True);sessions.append(s);image=s/'image.png';Image.fromarray(np.full((128,128,3),i,np.uint8)).save(image)
    (s/'sam2/prompts.json').write_text(json.dumps({'v/cam1':{'image':str(image),'positive_xy':[20,25],'box_xyxy':[0,0,127,127]}}))
   p=Predictor();run_sessions(sessions,p,'test');self.assertEqual(p.encodes,2)

 def test_depth_transform_once_per_camera_with_independent_masks(self):
  with tempfile.TemporaryDirectory() as folder:
   s=Path(folder);view=s/'v';view.mkdir();(s/'session.json').write_text(json.dumps({'views':[{'path':'v'}]}));meta={'turntable':{'angle_deg':30},'cameras':{c:{'depth_file':c+'.png','depth_scale_m':.001,'camera_info':{}} for c in ['cam1','cam2']}};(view/'metadata.json').write_text(json.dumps(meta));objects=[]
   for c in ['cam1','cam2']:cv2.imwrite(str(view/(c+'.png')),np.ones((20,20),np.uint16))
   for i in range(3):
    obj=s/'objects'/str(i);(obj/'sam2/v').mkdir(parents=True);objects.append(obj);mask=np.zeros((20,20),np.uint8);mask[i:i+12,:12]=255
    for c in ['cam1','cam2']:cv2.imwrite(str(obj/'sam2/v'/(c+'_mask.png')),mask)
   yy,xx=np.mgrid[:20,:20];points=np.stack([xx*.001,yy*.001,np.ones_like(xx)*.1],axis=-1);wc={'turntable':{'axis_point_table_m':[0,0,0]},'cameras':{c:{'T_table_optical':[]} for c in ['cam1','cam2']}}
   with mocked('shared_observations.depth_table',return_value=(points,np.ones((20,20),bool))) as transform:
    result=collect_plane_observations(s,objects,wc)
   self.assertEqual(transform.call_count,2)
   rotation=Rotation.from_euler('z',-30,degrees=True).as_matrix()
   for i,obj in enumerate(objects):
    expected=points[i:i+12,:12].reshape(-1,3)@rotation.T
    for c in ['cam1','cam2']:np.testing.assert_array_equal(result[obj.name][c][0][1],expected)

 def test_patch_builds_one_search_tree_for_many_candidate_centres(self):
  xx,yy=np.meshgrid(np.linspace(-.03,.03,45),np.linspace(-.03,.03,45));points=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,.1)]
  original=inclined_geometry.cKDTree
  with mocked.object(inclined_geometry,'cKDTree',wraps=original) as build:
   result=inclined_geometry.patch(points,[0,0,1],-.1)
  self.assertTrue(result['geometry_pass']);self.assertEqual(build.call_count,1)

if __name__=='__main__':unittest.main()
