import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from date_crops import collect_date_crops
from identity_routing import identity_routing

class DuplicateWorkTests(unittest.TestCase):
 def test_identity_mode_does_not_evaluate_colours_or_assign_destination(self):
  def forbidden():
   raise AssertionError('Colour pixels evaluated during date routing')
   yield
  r=identity_routing({'label':'초록 물체'}, {},identity_only=True,color_parts=forbidden())
  self.assertEqual(r['label'],'초록 물체');self.assertIsNone(r['destination_pad_id'])
  self.assertEqual(r['route_status'],'awaiting_expiry_date');self.assertFalse(r['color_routing_evaluated'])

 def test_legacy_colour_routing_and_crosscheck_still_work(self):
  policy={'routes':{'red':{'pad_id':6}}};red=np.tile([255,0,0],(20,1)).astype('uint8')
  r=identity_routing({'label':'빨간 원통'},policy,color_parts=[red]);self.assertEqual(r['destination_pad_id'],6)
  with self.assertRaises(RuntimeError):identity_routing({'label':'초록 물체'},policy,color_parts=[red])

 def test_shared_images_keep_independent_crop_pixels_and_scores(self):
  with tempfile.TemporaryDirectory() as folder:
   s=Path(folder);view=s/'view_001';view.mkdir();image=np.full((64,64,3),255,np.uint8)
   objects=[s/'objects'/f'object_{i}' for i in range(3)]
   for c in ['cam1','cam2']:cv2.imwrite(str(view/(c+'.png')),image)
   (view/'metadata.json').write_text(json.dumps({'cameras':{c:{'color_file':c+'.png'} for c in ['cam1','cam2']}}))
   for i,obj in enumerate(objects):
    d=obj/'sam2/view_001';d.mkdir(parents=True);mask=np.zeros((64,64),np.uint8);mask[10:22,5+i*18:17+i*18]=255
    for c in ['cam1','cam2']:cv2.imwrite(str(d/(c+'_mask.png')),mask)
   original=cv2.imread;rgb_reads=[]
   def counted(path,*args):
    if Path(path).parent==view:rgb_reads.append(path)
    return original(path,*args)
   with patch('date_crops.cv2.imread',side_effect=counted),patch('date_crops.cv2.cvtColor',wraps=cv2.cvtColor) as convert:
    result=collect_date_crops(s,{'views':[{'path':'view_001'}]},objects)
   self.assertEqual(len(rgb_reads),2);self.assertEqual(convert.call_count,2)
   for i,obj in enumerate(objects):
    self.assertEqual(len(result[obj.name]),2)
    for score,camera,evidence,crop in result[obj.name]:
     self.assertEqual(score,144);self.assertEqual(evidence,'view_001/'+camera)
     expected=np.full((22,22,3),128,np.uint8);expected[5:17,5:17]=255
     np.testing.assert_array_equal(crop,expected)

 def test_missing_mask_still_blocks_evidence(self):
  with tempfile.TemporaryDirectory() as folder:
   s=Path(folder);(s/'v').mkdir();cv2.imwrite(str(s/'v/image.png'),np.zeros((20,20,3),np.uint8))
   (s/'v/metadata.json').write_text(json.dumps({'cameras':{'cam1':{'color_file':'image.png'}}}))
   with self.assertRaisesRegex(ValueError,'mask missing'):
    collect_date_crops(s,{'views':[{'path':'v'}]},[s/'objects/object_1'])

if __name__=='__main__':unittest.main()
