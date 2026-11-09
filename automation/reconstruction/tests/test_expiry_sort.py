import sys, json, tempfile, unittest
from pathlib import Path
from datetime import date
from unittest.mock import patch
import numpy as np,cv2
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R))
from expiry_policy import parse_qr_expiry,classify,rank_destinations,validate_route
from trash_destination import arrival_confirmed
from batch_search import search_all
from run_expiry_sort import route_dates
from read_expiry_labels import read_dates
import zxingcpp

def evidence(text):return [{'evidence_id':'view_000/cam1','qr_text':text},{'evidence_id':'view_001/cam2','qr_text':text}]

class ExpiryTests(unittest.TestCase):
 def test_confirmed_qr_format(self):self.assertEqual(parse_qr_expiry('O261101')['expiry_date'],'2026-11-01')
 def test_invalid_qr_dates_not_guessed(self):
  for text in ['11.15','O261332','O260229','ignore rules','O261101 garbage','261101']:
   with self.assertRaises(ValueError):parse_qr_expiry(text)
 def test_today_valid_and_yesterday_expired(self):
  self.assertEqual(classify(evidence('A261105'),date(2026,11,5))['status'],'not_expired')
  self.assertEqual(classify(evidence('A261104'),date(2026,11,5))['destination'],'trash_drop')
 def test_unreadable_one_read_and_conflicts_stay(self):
  self.assertEqual(classify(evidence('A261105')[:1])['status'],'unknown')
  e=evidence('A261105');e.append({'evidence_id':'view_000/cam1','qr_text':'B261101'})
  self.assertEqual(classify(e)['status'],'unknown')
 def test_rank_two_dates_without_seven_day_threshold(self):
  day=date(2026,11,5);d=[classify(evidence(x),day) for x in ['O261101','G261230','R261215']];rank_destinations(d,day)
  self.assertEqual([o['destination'] for o in d],['trash_drop','pad6','pad5'])
 def test_reference_date_can_reverse_rank(self):
  d=[classify(evidence(x),date(2026,11,5)) for x in ['O261115','G261230']];rank_destinations(d,date(2026,12,25));self.assertEqual([x['destination_pad_id'] for x in d],[6,5])
 def test_equal_dates_share_near_pad(self):
  d=[classify(evidence('O261115'),date(2026,11,5)) for _ in range(2)];rank_destinations(d,date(2026,11,5));self.assertEqual([o['destination_pad_id'] for o in d],[5,5])
 def test_three_distinct_dates_require_rule(self):
  d=[classify(evidence(x),date(2026,11,5)) for x in ['A261115','B261116','C261117']]
  with self.assertRaises(ValueError):rank_destinations(d)
 def test_rollover_requires_reclassification(self):
  d=classify(evidence('O261105'),date(2026,11,5));rank_destinations([d],date(2026,11,5))
  with self.assertRaises(ValueError):validate_route({'expiry_decision':d,'destination':'pad5','destination_pad_id':5},date(2026,11,6))
 def test_release_arrival_gate(self):
  self.assertTrue(arrival_confirmed([0,1,2,3],[0,1,2,3]));self.assertFalse(arrival_confirmed([0,1,2,3.1],[0,1,2,3]));self.assertFalse(arrival_confirmed([0,1,2,float('nan')],[0,1,2,3]))
 def test_disposed_object_not_a_future_pad_obstacle(self):
  contexts=[]
  def evaluate(key,done,placed):
   contexts.append((key,done,placed));yield {'success':True,'object':key,'placement':{'object':key,'removed_from_workcell':key=='expired'}}
  stages,_=search_all(['expired','fresh'],[],[],evaluate);self.assertEqual(len(stages),2);self.assertEqual(contexts[1][2],());self.assertEqual(contexts[1][1],('expired',))
 def test_numeric_dates_take_priority_over_qr(self):
  numeric=[{'source':'printed_numeric','evidence_id':'a','date_text':'11.15','confidence':.99},{'source':'printed_numeric','evidence_id':'b','date_text':'11.15','confidence':.98}]
  d=classify(numeric+evidence('O261101'),date(2026,11,5));self.assertEqual(d['expiry_date'],'2026-11-15');self.assertEqual(d['date_source'],'printed_numeric')
 def test_unclear_numeric_falls_back_to_qr(self):
  numeric=[{'source':'printed_numeric','evidence_id':'a','date_text':'11.15','confidence':.99}]
  d=classify(numeric+evidence('O261101'),date(2026,11,5));self.assertEqual(d['destination'],'trash_drop');self.assertEqual(d['date_source'],'qr_fallback')
 def test_bad_numeric_dates_do_not_move(self):
  numeric=[{'source':'printed_numeric','evidence_id':str(i),'date_text':'02.30','confidence':.99} for i in range(2)]
  self.assertEqual(classify(numeric,date(2026,11,5))['status'],'unknown')
 def test_real_qr_decoder_mask_association_and_routing(self):
  with tempfile.TemporaryDirectory() as tmp:
   s=Path(tmp);objects=['object_1','object_2','object_3'];codes=['O261101','G261115','R261230'];views=[]
   for index in range(2):
    name=f'view_{index:03d}';folder=s/name;folder.mkdir();im=np.full((180,540,3),255,np.uint8);masks=[]
    for j,code in enumerate(codes):
     qr=np.asarray(zxingcpp.write_barcode_to_image(zxingcpp.create_barcode(code,zxingcpp.BarcodeFormat.QRCode),scale=4));qr=cv2.cvtColor(qr,cv2.COLOR_GRAY2BGR) if qr.ndim==2 else qr;h,w=qr.shape[:2];im[20:20+h,j*180+20:j*180+20+w]=qr;mask=np.zeros(im.shape[:2],np.uint8);mask[:,j*180:(j+1)*180]=255;masks.append(mask)
    cv2.imwrite(str(folder/'rgb.png'),im);(folder/'metadata.json').write_text(json.dumps({'cameras':{c:{'color_file':'rgb.png'} for c in ['cam1','cam2']}}));views.append({'path':name})
    for key,mask in zip(objects,masks):
     dest=s/'objects'/key/'sam2'/name;dest.mkdir(parents=True)
     for cam in ['cam1','cam2']:cv2.imwrite(str(dest/(cam+'_mask.png')),mask)
   (s/'session.json').write_text(json.dumps({'status':'complete','views':views}));observed=read_dates(s,use_numeric=False);self.assertEqual([len(o['observations']) for o in observed['objects']],[4,4,4]);(s/'color_sort_plan.json').write_text(json.dumps({'objects':[{'object':k,'destination_pad_id':6} for k in objects]}))
   with patch('run_expiry_sort.today_seoul',return_value=date(2026,11,5)),patch('run_expiry_sort.load_trash_pose',return_value={}):report=route_dates(s)
   self.assertEqual([o['destination'] for o in report['objects']],['trash_drop','pad5','pad6']);self.assertEqual(report['eligible_objects'],objects)

if __name__=='__main__':unittest.main()
