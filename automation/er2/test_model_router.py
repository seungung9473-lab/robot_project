"""Offline routing checks. No real API calls or robot access."""
import json,unittest
from pathlib import Path
from unittest import mock
import model_router
from model_router import generate,APIError,InvalidModelOutput,BlockedModelOutput
P='gemini-robotics-er-2-preview';F='gemini-3.8-flash'
BASE=json.loads((Path(model_router.__file__).resolve().parent/'model_policy.json').read_text())
ER2_POLICY={**BASE,'primary':P,'fallback':F,'fallback_enabled':True}
FLASH_ONLY_POLICY={**BASE,'primary':F,'fallback':F,'fallback_enabled':False,'retry_same_model_once':False}
def response(model,value=True):return {'modelVersion':model,'candidates':[{'finishReason':'STOP'}],'value':value}
def validate(value):
 if not value.get('value'):raise ValueError('Invalid output')
 return True
class RouterTests(unittest.TestCase):
 def setUp(self):
  patcher=mock.patch('model_router.policy',return_value=ER2_POLICY);patcher.start();self.addCleanup(patcher.stop)
 def route(self,results):
  calls=[]
  def transport(key,method,path,payload):
   calls.append(path);item=results[len(calls)-1]
   if isinstance(item,Exception):raise item
   return item
  return calls,lambda:generate('offline-test-key',{},validate,transport=transport,test_context='offline_unit_test')
 def test_primary_only(self):
  calls,run=self.route([response(P)]);_,_,r=run();self.assertEqual(len(calls),1);self.assertFalse(r['fallback_used'])
 def test_missing_and_server_error(self):
  for status in [404,408,410,500,502,503,504]:
   calls,run=self.route([APIError('test',status),response(F)]);_,_,r=run();self.assertEqual(len(calls),2);self.assertEqual(r['model_used'],F)
 def test_shared_account_errors_stop(self):
  for status in [400,401,402,403,429]:
   calls,run=self.route([APIError('test',status)])
   with self.assertRaises(APIError):run()
   self.assertEqual(len(calls),1)
 def test_invalid_primary_output_validates_fallback(self):
  calls,run=self.route([response(P,False),response(F,False)])
  with self.assertRaises(InvalidModelOutput):run()
  self.assertEqual(len(calls),2)
 def test_blocked_response_not_bypassed(self):
  data=response(P);data['candidates'][0]['finishReason']='SAFETY';calls,run=self.route([data])
  with self.assertRaises(BlockedModelOutput):run()
  self.assertEqual(len(calls),1)
 def test_timeout_switch(self):
  calls,run=self.route([TimeoutError(),response(F)]);run();self.assertEqual(len(calls),2)
 def test_no_repeat_loop(self):
  calls,run=self.route([APIError('test',503),APIError('test',503)])
  with self.assertRaises(APIError):run()
  self.assertEqual(len(calls),2)
 def test_shared_network_failure_stops(self):
  calls,run=self.route([RuntimeError('Network unavailable')])
  with self.assertRaises(RuntimeError):run()
  self.assertEqual(len(calls),1)
if __name__=='__main__':unittest.main()

class FlashOnlyTests(unittest.TestCase):
 def run_with(self,results,policy=FLASH_ONLY_POLICY):
  calls=[]
  def transport(key,method,path,payload):
   calls.append(path);item=results[len(calls)-1]
   if isinstance(item,Exception):raise item
   return item
  with mock.patch('model_router.policy',return_value=policy):
   return calls,generate('offline-test-key',{},validate,transport=transport,test_context='offline_unit_test')
 def test_default_policy_file_is_flash_only(self):
  self.assertEqual((BASE['primary'],BASE['fallback'],BASE['fallback_enabled']),(F,F,False))
  model_router.policy()  # 저장된 정책이 라우터 검증을 통과해야 한다
 def test_flash_only_never_calls_er2(self):
  calls,(_,_,r)=self.run_with([response(F)])
  self.assertEqual(calls,[F+':generateContent']);self.assertEqual(r['model_used'],F);self.assertFalse(r['fallback_used'])
 def test_flash_only_failure_does_not_retry_or_switch(self):
  calls=[]
  def transport(key,method,path,payload):calls.append(path);raise APIError('test',503)
  with mock.patch('model_router.policy',return_value=FLASH_ONLY_POLICY):
   with self.assertRaises(APIError):generate('offline-test-key',{},validate,transport=transport,test_context='offline_unit_test')
  self.assertEqual(len(calls),1)
 def test_flash_policy_with_fallback_enabled_is_rejected(self):
  bad={**FLASH_ONLY_POLICY,'fallback_enabled':True}
  with mock.patch.object(model_router,'ROOT') as root:
   root.__truediv__.return_value.read_text.return_value=json.dumps(bad)
   with self.assertRaises(ValueError):model_router.policy()

RETRY_POLICY={**FLASH_ONLY_POLICY,'retry_same_model_once':True}
class FlashRetryTests(unittest.TestCase):
 def run_with(self,results):
  calls=[]
  def transport(key,method,path,payload):
   calls.append(path);item=results[len(calls)-1]
   if isinstance(item,Exception):raise item
   return item
  with mock.patch('model_router.policy',return_value=RETRY_POLICY):
   return calls,transport
 def go(self,transport):
  with mock.patch('model_router.policy',return_value=RETRY_POLICY):
   return generate('offline-test-key',{},validate,transport=transport,test_context='offline_unit_test')
 def test_timeout_then_success_retries_same_model_once(self):
  calls,transport=self.run_with([TimeoutError(),response(F)])
  _,_,r=self.go(transport);self.assertEqual(calls,[F+':generateContent']*2);self.assertTrue(r['success'])
 def test_three_timeouts_stop_after_exactly_three_requests(self):
  calls,transport=self.run_with([TimeoutError(),TimeoutError(),TimeoutError(),response(F)])
  with self.assertRaises(TimeoutError):self.go(transport)
  self.assertEqual(len(calls),3)
 def test_two_timeouts_then_success_is_accepted(self):
  calls,transport=self.run_with([TimeoutError(),TimeoutError(),response(F)])
  _,_,r=self.go(transport);self.assertEqual(len(calls),3);self.assertTrue(r['success'])
 def test_server_error_is_retried_but_credit_and_quota_errors_are_not(self):
  calls,transport=self.run_with([APIError('test',503),response(F)]);self.go(transport);self.assertEqual(len(calls),2)
  for status in [400,401,402,403,429]:
   calls,transport=self.run_with([APIError('test',status),response(F)])
   with self.assertRaises(APIError):self.go(transport)
   self.assertEqual(len(calls),1)
 def test_saved_policy_allows_one_retry(self):
  self.assertTrue(BASE.get('retry_same_model_once'));model_router.policy()

class RequestTimeoutTests(unittest.TestCase):
 def test_request_timeout_comes_from_policy_and_has_headroom(self):
  import check_connection
  self.assertEqual(check_connection.REQUEST_TIMEOUT_S,float(BASE['request_timeout_s']))
  self.assertGreaterEqual(check_connection.REQUEST_TIMEOUT_S,60)   # 45초 한도에서 실제 호출(약 30~40초)이 자주 실패했다
  self.assertNotIn('timeout=45',Path(check_connection.__file__).read_text())
