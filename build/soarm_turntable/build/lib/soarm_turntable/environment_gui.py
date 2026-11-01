"""Read-only live environment setup. There are no torque/goto/stop calls here."""
import argparse,json,time,sys
from pathlib import Path
import cv2,numpy as np
from PyQt5.QtCore import Qt,QTimer,pyqtSignal,QPointF,QRectF
from PyQt5.QtGui import QImage,QPainter,QPen,QPolygonF
from PyQt5.QtWidgets import QApplication,QWidget,QLabel,QPushButton,QVBoxLayout,QHBoxLayout,QGridLayout,QComboBox,QSpinBox,QDoubleSpinBox,QScrollArea
import rclpy
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.convert import message_to_ordereddict
from .scan import Scanner
from .scan_core import select_bundle,stable_states
from .environment_core import foreground,background_median,validate_polygon,calibration_key,save_environment,load_environment

class Canvas(QWidget):
    clicked=pyqtSignal(int,int)
    def __init__(self):
        super().__init__();self.image=None;self.points=[];self.setMinimumSize(300,225);self.setMaximumSize(520,390);self.setMouseTracking(True)
    def set_frame(self,bgr,points=None):
        rgb=cv2.cvtColor(np.ascontiguousarray(bgr),cv2.COLOR_BGR2RGB);self.image=QImage(rgb.data,rgb.shape[1],rgb.shape[0],rgb.strides[0],QImage.Format_RGB888).copy();self.points=points or [];self.update()
    def image_rect(self):
        if self.image is None:return QRectF()
        size=self.image.size();size.scale(self.size(),Qt.KeepAspectRatio);return QRectF((self.width()-size.width())/2,(self.height()-size.height())/2,size.width(),size.height())
    def paintEvent(self,event):
        p=QPainter(self);p.fillRect(self.rect(),Qt.black)
        if self.image is None:p.setPen(Qt.white);p.drawText(self.rect(),Qt.AlignCenter,'영상 수신 대기');return
        rect=self.image_rect();p.drawImage(rect,self.image);p.setPen(QPen(Qt.cyan,2))
        points=[QPointF(rect.x()+x*rect.width()/self.image.width(),rect.y()+y*rect.height()/self.image.height()) for x,y in self.points]
        if len(points)>=2:p.drawPolyline(QPolygonF(points+[points[0]]))
        for point in points:p.drawEllipse(point,4,4)
    def mousePressEvent(self,event):
        rect=self.image_rect()
        if self.image is not None and event.button()==Qt.LeftButton and rect.contains(event.localPos()):
            self.clicked.emit(min(self.image.width()-1,int((event.x()-rect.x())*self.image.width()/rect.width())),min(self.image.height()-1,int((event.y()-rect.y())*self.image.height()/rect.height())))

class EnvironmentWindow(QWidget):
    def __init__(self,node=None,replay=None,output=None):
        super().__init__();self.node=node;self.replay=replay;self.output=Path(output).expanduser();self.frames={};self.cameras={c:{'polygon':[],'background':None,'color':None,'info':None} for c in ('cam1','cam2')};self.collect=None;self.drawing=None;self.last_bundle=None;self.source_stamp={};self.saved_path=None;self.last_live_update=0.
        self.setWindowTitle('SO-ARM4 환경 세팅 · 모터 명령 없음');self.resize(1320,920);root=QVBoxLayout(self)
        guide=QLabel('① 케이블을 회전 범위 밖으로 고정 → ② 카메라별 물체가 보일 영역 지정 → ③ 물체 없이 빈 배경 수집 → ④ 물체를 올려 초록색 후보 확인 → ⑤ 환경 저장\n실제 케이블을 영상 필터로 안전하게 만드는 기능은 아닙니다. 카메라/테이블을 옮기면 배경과 영역을 다시 설정하세요.');guide.setWordWrap(True);root.addWidget(guide)
        self.health=QLabel('영상·턴테이블 수신 대기');root.addWidget(self.health);self.status=QLabel('선택한 카메라 원본 화면을 클릭해 다각형 꼭짓점을 지정합니다.');self.status.setWordWrap(True);root.addWidget(self.status)
        row=QHBoxLayout();self.selector=QComboBox();self.selector.addItems(['cam1','cam2']);row.addWidget(self.selector)
        def button(text,fn):
            b=QPushButton(text);b.clicked.connect(fn);row.addWidget(b);return b
        self.draw_button=button('영역 그리기',self.begin_polygon);self.finish_button=button('영역 확정',self.finish_polygon);self.clear_button=button('선택 영역 초기화',self.clear_polygon);self.background_button=button('빈 배경 10장 수집',self.start_background);self.save_button=button('환경 저장',self.save);root.addLayout(row)
        filters=QHBoxLayout();self.threshold=QDoubleSpinBox();self.threshold.setRange(1,200);self.threshold.setValue(10);self.threshold.setSuffix(' mm');filters.addWidget(QLabel('배경보다 가까운 깊이 차이'));filters.addWidget(self.threshold)
        self.near=QSpinBox();self.near.setRange(1,9999);self.near.setValue(150);self.far=QSpinBox();self.far.setRange(2,10000);self.far.setValue(2000);filters.addWidget(QLabel('유효 깊이 최소/최대(mm)'));filters.addWidget(self.near);filters.addWidget(self.far)
        self.area=QSpinBox();self.area.setRange(1,1000000);self.area.setValue(100);filters.addWidget(QLabel('최소 덩어리(px)'));filters.addWidget(self.area);root.addLayout(filters)
        scroll=QScrollArea();scroll.setWidgetResizable(True);body=QWidget();grid=QGridLayout(body);self.canvases={};self.stats={}
        for col,title in enumerate(['원본 · 영역 지정은 여기 클릭','깊이 · 검정은 깊이 없음','물체 후보 · 초록=전경, 분홍=깊이 미확인']):grid.addWidget(QLabel(title),0,col)
        for index,camera in enumerate(('cam1','cam2')):
            for col,kind in enumerate(('color','depth','mask')):
                canvas=Canvas();self.canvases[camera+'_'+kind]=canvas;grid.addWidget(canvas,index*3+1,col)
                if kind=='color':canvas.clicked.connect(lambda x,y,c=camera:self.add_point(c,x,y))
            self.stats[camera]=QLabel(camera+' 수신 대기');self.stats[camera].setWordWrap(True);grid.addWidget(self.stats[camera],index*3+2,0,1,3)
        scroll.setWidget(body);root.addWidget(scroll)
        self.mode=QLabel('재생 모드 · 실제 환경으로 적용 불가' if replay else '실시간 모드 · 영역/배경 필터는 2D 후보 추출이며 3D 보정/흡착면 판정은 아닙니다.');root.addWidget(self.mode)
        if replay:self.load_replay(replay)
        active=self.output/'active.json'
        if active.exists():
            try:self.load_config(active)
            except Exception as e:self.status.setText('기존 환경 로드 실패: '+str(e))
        self.timer=QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(120)
    def filters(self):return {'threshold_mm':self.threshold.value(),'near_mm':self.near.value(),'far_mm':self.far.value(),'min_area':self.area.value()}
    def load_replay(self,folder):
        folder=Path(folder);meta=json.loads((folder/'metadata.json').read_text())
        for cam,data in meta['cameras'].items():
            color=cv2.imread(str(folder/data['color_file']));depth=np.load(folder/data['depth_file'],allow_pickle=False) if data['depth_file'].endswith('.npy') else cv2.imread(str(folder/data['depth_file']),cv2.IMREAD_UNCHANGED)
            self.frames[cam]=(color,depth.astype(np.float32)*data['depth_scale_m']*1000,data['camera_info'])
        self.last_bundle=meta
    def load_config(self,path):
        data,p=load_environment(path,allow_replay=bool(self.replay));self.saved_path=p
        for cam,v in data['cameras'].items():self.cameras[cam]={'polygon':v['polygon'],'background':v['background'],'color':cv2.imread(str(p.parent/v['color_file'])),'info':v['camera_info']}
        for name,widget in [('threshold_mm',self.threshold),('near_mm',self.near),('far_mm',self.far),('min_area',self.area)]:widget.setValue(data['filters'][name])
        self.status.setText('기존 환경 로드: '+str(p))
    def begin_polygon(self):
        cam=self.selector.currentText()
        if cam not in self.frames:self.status.setText('먼저 영상 수신을 확인하세요');return
        if self.collect:self.status.setText('배경 수집 중에는 영역을 바꿀 수 없습니다');return
        self.drawing=cam;self.cameras[cam]['polygon']=[];self.status.setText(cam+' 원본 화면에서 물체가 보일 영역 꼭짓점을 순서대로 클릭하고 영역 확정을 누르세요')
    def add_point(self,cam,x,y):
        if self.drawing==cam:self.cameras[cam]['polygon'].append([x,y])
    def finish_polygon(self):
        cam=self.drawing or self.selector.currentText()
        try:
            image=self.frames[cam][0];validate_polygon(self.cameras[cam]['polygon'],image.shape[1],image.shape[0]);self.drawing=None;self.status.setText(cam+' 영역 확정')
        except Exception as e:self.status.setText(str(e))
    def clear_polygon(self):
        if self.collect:return
        self.cameras[self.selector.currentText()]['polygon']=[];self.drawing=None;self.status.setText('선택 영역 초기화 · 배경은 유지, 새 영역을 지정하세요')
    def start_background(self):
        try:
            if self.drawing:raise ValueError('영역 확정을 먼저 누르세요')
            for cam in ('cam1','cam2'):
                color,depth,info=self.frames[cam];validate_polygon(self.cameras[cam]['polygon'],color.shape[1],color.shape[0])
            if self.node:
                self.node.state()
                if not stable_states(list(self.node.states),time.monotonic()):raise ValueError('턴테이블 상태/정지를 먼저 확인하세요. 읽기 전용 브리지가 필요합니다')
            self.collect={'cam1':[],'cam2':[],'last':{},'start':time.monotonic(),'info':{c:calibration_key(self.frames[c][2]) for c in ('cam1','cam2')}};self.background_button.setEnabled(False);self.status.setText('빈 턴테이블 배경 수집 중 · 카메라/물체를 움직이지 마세요')
        except Exception as e:self.status.setText(str(e))
    def save(self):
        try:
            if self.collect or self.drawing:raise ValueError('영역 지정/배경 수집을 먼저 완료하세요')
            if self.node and time.monotonic()-self.last_live_update>1:raise ValueError('현재 영상 수신을 먼저 확인하세요')
            for cam in ('cam1','cam2'):
                if cam not in self.frames or self.cameras[cam]['background'] is None:raise ValueError(cam+' 빈 배경을 먼저 수집하세요')
                if calibration_key(self.frames[cam][2])!=calibration_key(self.cameras[cam]['info']):raise ValueError('카메라 내부행렬이 변경됐습니다. 배경을 다시 저장하세요')
            self.saved_path=save_environment(self.output,self.cameras,self.filters(),'replay' if self.replay else 'live');self.status.setText('저장 완료: '+str(self.saved_path))
        except Exception as e:self.status.setText('저장 실패: '+str(e))
    def acquire(self):
        if not self.node:return
        for _ in range(30):rclpy.spin_once(self.node,timeout_sec=0)
        bundle=select_bundle(self.node.buffers,self.node.get_clock().now().nanoseconds-500_000_000,time.monotonic())
        if bundle is None:raise RuntimeError('두 카메라의 최신 컬러/정렬깊이/내부행렬 대기 · 카메라 실행/USB3 확인')
        stamps=[]
        for cam in ('cam1','cam2'):
            color=bundle[cam+'_color'][1];depth=bundle[cam+'_depth'][1];info=bundle[cam+'_info'][1]
            if (color.width,color.height)!=(depth.width,depth.height) or (color.width,color.height)!=(info.width,info.height):raise ValueError('영상/내부행렬 해상도 불일치')
            rgb=self.node.bridge.imgmsg_to_cv2(color,'bgr8');raw=self.node.bridge.imgmsg_to_cv2(depth,'passthrough')
            if depth.encoding=='16UC1':mm=raw.astype(np.float32)
            elif depth.encoding=='32FC1':mm=raw.astype(np.float32)*1000
            else:raise ValueError('지원하지 않는 깊이 인코딩')
            self.frames[cam]=(rgb,mm,message_to_ordereddict(info));self.source_stamp[cam]=depth.header.stamp.sec*10**9+depth.header.stamp.nanosec;stamps.append(self.source_stamp[cam])
        self.last_live_update=time.monotonic()
        self.health.setText('카메라 정상 · 깊이 시각 차이 %.1fms · '%((max(stamps)-min(stamps))/1e6)+self.table_status())
    def table_status(self):
        try:
            s=self.node.state();return '턴테이블 %d count / 토크 %s / %s / %.1fV'%(s['position_count'],'ON' if s['torque'] else 'OFF','이동 중' if s['moving'] else '정지',s['voltage'])
        except Exception:return '턴테이블 상태 없음 · 영상 확인 가능, 배경 수집에는 읽기 전용 브리지 필요'
    def tick(self):
        try:
            self.acquire()
            if self.collect:
                if time.monotonic()-self.collect['start']>10:raise RuntimeError('배경 수집 시간 초과')
                if self.node and not stable_states(list(self.node.states),time.monotonic()):raise RuntimeError('배경 수집 중 턴테이블 움직임/통신 지연')
                for cam,(color,depth,info) in self.frames.items():
                    stamp=self.source_stamp.get(cam,time.monotonic_ns())
                    if self.collect['info'][cam]!=calibration_key(info):raise RuntimeError('배경 수집 중 카메라 설정 변경')
                    if len(self.collect[cam])<10 and self.collect['last'].get(cam)!=stamp:self.collect[cam].append(depth.copy());self.collect['last'][cam]=stamp
                if all(len(self.collect[c])>=10 for c in ('cam1','cam2')):
                    for cam in ('cam1','cam2'):
                        color,depth,info=self.frames[cam];self.cameras[cam].update(background=background_median(self.collect[cam]),color=color.copy(),info=info)
                    self.collect=None;self.background_button.setEnabled(True);self.status.setText('빈 배경 수집 완료 · 이제 물체를 올리고 초록색 후보를 확인한 뒤 환경 저장을 누르세요')
                else:self.status.setText('빈 배경 수집: cam1 %d/10 · cam2 %d/10'%(len(self.collect['cam1']),len(self.collect['cam2'])))
            for cam,(color,depth,info) in self.frames.items():self.render(cam,color,depth,info)
        except Exception as e:
            self.health.setText(str(e))
            if self.collect:self.collect=None;self.background_button.setEnabled(True);self.status.setText('배경 수집 중단: '+str(e))
    def render(self,cam,color,depth,info):
        spec=self.cameras[cam];self.canvases[cam+'_color'].set_frame(color,spec['polygon'])
        scaled=np.clip((depth-self.near.value())/max(1,self.far.value()-self.near.value()),0,1);heat=cv2.applyColorMap((np.nan_to_num(scaled)*255).astype(np.uint8),cv2.COLORMAP_TURBO);heat[~np.isfinite(depth)|(depth<=0)]=0;self.canvases[cam+'_depth'].set_frame(heat)
        preview=(color*.25).astype(np.uint8);message=cam+' · 유효 깊이 %.1f%%'%(100*np.mean(np.isfinite(depth)&(depth>0)))
        if spec['background'] is not None and len(spec['polygon'])>=3:
            try:
                if calibration_key(info)!=calibration_key(spec['info']):raise ValueError('카메라 설정 변경 · 배경 다시 수집')
                mask,roi=foreground(depth,spec['background'],spec['polygon'],**self.filters());preview[roi>0]=color[roi>0];preview[mask>0]=(color[mask>0]*.4+np.array([0,150,0])).clip(0,255).astype(np.uint8)
                unknown=(roi>0)&((depth<=0)|~np.isfinite(depth)|(spec['background']<=0));preview[unknown]=[180,0,180];message+=' · 영역 깊이 미확인 %.1f%% · 후보 %dpx'%(100*unknown.sum()/max(1,(roi>0).sum()),(mask>0).sum())
            except Exception as e:message+=' · '+str(e)
        else:message+=' · 영역/빈 배경을 설정하세요'
        self.canvases[cam+'_mask'].set_frame(preview);self.stats[cam].setText(message)

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',default=str(Path.home()/'.ros/soarm4/environment'));p.add_argument('--replay',help='저장된 view_NNN 폴더로 조작 연습. 실제 환경으로 적용 불가');a=p.parse_args(remove_ros_args(args=sys.argv)[1:]);app=QApplication(sys.argv[:1]);node=None
    if not a.replay:rclpy.init();node=Scanner()
    window=EnvironmentWindow(node=node,replay=a.replay,output=a.output);window.show()
    try:app.exec_()
    finally:
        if node:node.destroy_node();rclpy.try_shutdown()
