#!/usr/bin/env python3
import argparse,sys,time,json,uuid
import rclpy
from std_srvs.srv import Trigger
from std_msgs.msg import String
from pick_place_core import SECTIONS

def main():
 p=argparse.ArgumentParser();p.add_argument('command',choices=['insert','insert_grip','insert_release','replace','delete','move','clear','list','check','run','stop','grip','release']);p.add_argument('target',nargs='?');p.add_argument('--index',type=int,default=0);p.add_argument('--delta',type=int,choices=[-1,1],default=1);a=p.parse_args()
 if a.command in ['insert','insert_grip','insert_release','replace','delete','move'] and a.target not in SECTIONS:p.error('구간: '+', '.join(SECTIONS))
 rclpy.init();n=rclpy.create_node('pick_place_command')
 try:
  if a.command in ['list','check','run','stop','grip','release']:
   path=('/soarm4/vacuum/'+a.command) if a.command in ['grip','release'] else ('/soarm4/stop_home' if a.command=='stop' else '/soarm4/pick_place/'+('save_'+a.target if a.command=='save' else a.command));c=n.create_client(Trigger,path)
   if not c.wait_for_service(timeout_sec=3):print('서비스 없음: '+path);return 1
   f=c.call_async(Trigger.Request())
   try:rclpy.spin_until_future_complete(n,f)
   except KeyboardInterrupt:
    if a.command in ['run','check']:
     c=n.create_client(Trigger,'/soarm4/stop_home')
     if c.wait_for_service(timeout_sec=1):rclpy.spin_until_future_complete(n,c.call_async(Trigger.Request()),timeout_sec=2)
    return 1
   r=f.result();print(r.message);return 0 if r.success else 1
  request=uuid.uuid4().hex;responses=[];n.create_subscription(String,'/soarm4/pick_place/edit_result',lambda m:responses.append(json.loads(m.data)),10);pub=n.create_publisher(String,'/soarm4/pick_place/edit',10);end=time.monotonic()+3
  while pub.get_subscription_count()<1 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
  if pub.get_subscription_count()!=1:print('편집 노드가 없거나 중복입니다');return 1
  pub.publish(String(data=json.dumps({'request_id':request,'action':a.command,'section':a.target,'index':a.index,'delta':a.delta})));end=time.monotonic()+10
  while time.monotonic()<end:
   rclpy.spin_once(n,timeout_sec=.1)
   for response in responses:
    if response.get('request_id')==request:print(response['message']);return 0 if response['success'] else 1
  print('응답 없음. list로 결과를 확인하세요.');return 1
 finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':sys.exit(main())
