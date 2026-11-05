import json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import inclined_runtime as runtime
class FixedContactTests(unittest.TestCase):
 def test_verification_preserves_predicted_plane(self):
  point=np.array([1.,2.,.1]);normal=np.array([0.,0.,1.]);cloud=np.tile(point,(12,12,1));meta={'cameras':{c:{'depth_file':'d.png','depth_scale_m':.001,'camera_info':{}} for c in ['cam1','cam2']}}
  with tempfile.TemporaryDirectory() as d:
   Path(d,'metadata.json').write_text(json.dumps(meta))
   with patch.object(runtime.cv2,'imread',return_value=np.ones((12,12),np.uint16)),patch.object(runtime,'depth_table',return_value=(cloud,np.ones((12,12),bool))),patch.object(runtime,'patch',return_value={'geometry_pass':True}) as check:
    result=runtime.fresh_face_check(d,{'turntable':{'top_height_m':.05},'cameras':{c:{'T_table_optical':[]} for c in ['cam1','cam2']}},point,normal)
    for call in check.call_args_list:
     np.testing.assert_array_equal(call.args[1],normal);self.assertEqual(call.args[2],-.1);np.testing.assert_array_equal(call.kwargs['center_point'],point)
    self.assertFalse(result['contact_changed']);self.assertFalse(result['normal_changed'])
if __name__=='__main__':unittest.main()
