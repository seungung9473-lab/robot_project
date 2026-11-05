"""Fresh Gemini identities and measured stationary reference; no robot commands."""
import sys,json,subprocess,shutil,cv2,numpy as np,yaml
from pathlib import Path
from placement_geometry import depth_table
from inclined_runtime import depth_scale
s=Path(sys.argv[1]).resolve();R=Path(__file__).resolve().parents[1];image=s/'view_000/cam1_color.png';out=subprocess.check_output([sys.executable,R/'er2/select_object.py','prepare','--image',image,'--crop','245','150','460','360','--all','--instruction','턴테이블 위 각각의 물체를 실제 색상(회색/초록색/빨간색)과 모양으로 식별하세요. 각 물체의 보이는 부분 안쪽에 점을 지정하세요.'],text=True).strip();subprocess.run([sys.executable,R/'er2/select_object.py','select',out],check=True);det=json.loads((Path(out)/'selection_result.json').read_text());assert 1<=len(det['objects'])<=8
ref=s/'stationary_reference';ref.mkdir(exist_ok=True);cfg=Path.home()/'robot_ws/src/soarm4_workcell/config/workcell.yaml';wc=yaml.safe_load(cfg.read_text());shutil.copy2(cfg,ref/'workcell_snapshot.yaml');md=json.loads((s/'view_000/metadata.json').read_text())['cameras']['cam1'];raw=cv2.imread(str(s/'view_000'/md['depth_file']),-1);P,valid=depth_table(raw,depth_scale(md,raw),md['camera_info'],wc['cameras']['cam1']['T_table_optical']);records=[]
for obj in det['objects']:
 x,y=obj['point_full_image_xy'];pts=P[max(0,y-2):y+3,max(0,x-2):x+3][valid[max(0,y-2):y+3,max(0,x-2):x+3]];assert len(pts)>=5;label=obj['label'];color='green' if '초록' in label else ('red' if '빨' in label else ('gray' if '회색' in label else None));assert color is not None,'Unassigned colour; do not guess';records.append({'object':'object_'+str(obj['object_id']),'label':label,'color':color,'reference_center_xy':np.median(pts,axis=0)[:2].tolist(),'destination_pad_id':5 if color=='green' else 6})
(ref/'color_sort_plan.json').write_text(json.dumps({'objects':records},ensure_ascii=False,indent=2));(s/'gemini_detection.json').write_text(json.dumps(det,ensure_ascii=False,indent=2));print('STATIONARY REFERENCES',records,flush=True)
