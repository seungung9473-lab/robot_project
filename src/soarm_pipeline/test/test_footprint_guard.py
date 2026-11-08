import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'automation/reconstruction'))
from depth_mask import erode_depth_mask
from pad_occupancy import inflate_polygon,FOOTPRINT_MARGIN_M
def test_erosion_removes_edge_pixels():
    m=np.zeros((60,60),bool);m[10:50,10:50]=True
    e=erode_depth_mask(m,3);assert e.sum()==34*34 and not e[10,10] and e[13,13]
def test_small_object_keeps_original_mask():
    m=np.zeros((60,60),bool);m[20:26,20:26]=True
    assert (erode_depth_mask(m,3)==m).all()
def test_inflate_grows_by_margin_not_less():
    sq=np.array([[0,0],[.1,0],[.1,.1],[0,.1]])
    big=inflate_polygon(sq,FOOTPRINT_MARGIN_M)
    assert big[:,0].min()<=-FOOTPRINT_MARGIN_M+1e-9 and big[:,0].max()>=.1+FOOTPRINT_MARGIN_M-1e-9
    assert big[:,0].max()<=.1+FOOTPRINT_MARGIN_M*1.03
def test_inflate_zero_margin_is_identity():
    sq=np.array([[0,0],[1,0],[1,1],[0,1]],float);assert (inflate_polygon(sq,0)==sq).all()
