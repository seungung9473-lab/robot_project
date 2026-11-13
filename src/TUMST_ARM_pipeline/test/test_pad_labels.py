import json,sys
from pathlib import Path
import numpy as np
import pytest
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from pad_labels import normalize_label,assign,parse_response,table_xy,build_payload
from expiry_policy import rank_destinations

PADS={5:(1.251,0.367),6:(1.246,0.667)}
def det(cam,label,xy):return {'cam':cam,'label':label,'pixel':[0,0],'xy':xy}

def test_label_text_variants():
    assert normalize_label('할인')=='discount' and normalize_label(' 할 인 ')=='discount' and normalize_label('정 규.')=='regular'
    assert normalize_label('11.15') is None and normalize_label('') is None and normalize_label(None) is None

def test_assign_normal_layout():
    r=assign([det('cam1','discount',[1.25,0.38]),det('cam1','regular',[1.24,0.69])],PADS)
    assert (r['discount_pad_id'],r['regular_pad_id'])==(5,6)

def test_assign_follows_swapped_stickers():
    r=assign([det('cam1','discount',[1.24,0.69]),det('cam1','regular',[1.25,0.38])],PADS)
    assert (r['discount_pad_id'],r['regular_pad_id'])==(6,5)

def test_two_cameras_may_cover_one_sticker_each():
    r=assign([det('cam1','discount',[1.25,0.38]),det('cam2','regular',[1.24,0.69])],PADS)
    assert (r['discount_pad_id'],r['regular_pad_id'])==(5,6)

def test_missing_sticker_stops():
    with pytest.raises(RuntimeError,match='정규'):assign([det('cam1','discount',[1.25,0.38])],PADS)

def test_same_pad_or_conflicting_pads_stop():
    with pytest.raises(RuntimeError,match='same pad'):assign([det('cam1','discount',[1.25,0.38]),det('cam1','regular',[1.25,0.35])],PADS)
    with pytest.raises(RuntimeError,match='different pads'):assign([det('cam1','discount',[1.25,0.38]),det('cam2','discount',[1.24,0.69]),det('cam1','regular',[1.24,0.66])],PADS)

def test_sticker_far_from_every_pad_or_without_depth_is_ignored():
    with pytest.raises(RuntimeError,match='할인'):assign([det('cam1','discount',[1.6,0.1]),det('cam1','regular',[1.24,0.69])],PADS)
    with pytest.raises(RuntimeError,match='할인'):assign([det('cam1','discount',None),det('cam1','regular',[1.24,0.69])],PADS)

def response(stickers):return {'candidates':[{'content':{'parts':[{'text':json.dumps({'stickers':stickers})}]}}]}
def test_parse_response_scales_points_and_drops_other_text():
    rows=parse_response(response([{'text':'할 인','point_yx':[500,250]},{'text':'11.15','point_yx':[100,100]}]),(480,640))
    assert rows==[('discount',(160,240))]
    with pytest.raises(ValueError):parse_response(response([{'text':'정규','point_yx':[1200,10]}]),(480,640))

def test_table_xy_from_depth_with_identity_camera():
    raw=np.full((480,640),500,np.uint16)    # 0.5 m everywhere (valid range is 0.08-0.9 m)
    info={'k':[500,0,320,0,500,240,0,0,1],'d':[0,0,0,0,0]};T=np.eye(4)
    xy=table_xy(raw,.001,info,T,(320,240))
    assert abs(xy[0])<1e-6 and abs(xy[1])<1e-6
    assert table_xy(np.zeros((480,640),np.uint16),.001,info,T,(320,240)) is None

def test_payload_has_no_robot_tools_and_is_json_schema():
    p=build_payload(b'png');assert 'tools' not in p and p['generationConfig']['responseMimeType']=='application/json'

def decisions():
    return [{'object':'a','status':'not_expired','expiry_date':'2026-11-15'},{'object':'b','status':'not_expired','expiry_date':'2026-12-30'}]
def test_rank_defaults_keep_fixed_layout():
    from datetime import date
    d=rank_destinations(decisions(),date(2026,11,9))
    assert [(x['destination_pad_id'],x['pad_label']) for x in d]==[(5,'할인'),(6,'정규')]
def test_rank_follows_labels_when_pads_are_swapped():
    from datetime import date
    d=rank_destinations(decisions(),date(2026,11,9),{'discount':6,'regular':5})
    assert [(x['destination'],x['destination_pad_id'],x['pad_label']) for x in d]==[('pad6',6,'할인'),('pad5',5,'정규')]
def test_rank_rejects_bad_pad_labels():
    from datetime import date
    for bad in ({'discount':5,'regular':5},{'discount':5,'regular':7}):
        with pytest.raises(ValueError):rank_destinations(decisions(),date(2026,11,9),bad)
