import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'automation/reconstruction'))
from pad_occupancy import free_offsets,free_offsets_refined
CENTER=np.array([0.,0.,0.]);RADIUS=.05;OCC={'obstacles':[]}
def circle(r,cx,cy,n=64):
    a=np.linspace(0,2*np.pi,n,endpoint=False);return np.c_[cx+r*np.cos(a),cy+r*np.sin(a)]
def coarse_grid():
    t=np.arange(-25,26)*.002;return np.array([(x,y) for x in t for y in t])
def test_tight_shape_found_only_on_fine_grid():
    poly=circle(.0478,.0013,-.0007)  # 0.2 mm slack, off the 2 mm lattice
    assert len(free_offsets(poly,coarse_grid(),CENTER,RADIUS,OCC))==0
    free,fine=free_offsets_refined(poly,coarse_grid(),CENTER,RADIUS,OCC)
    assert fine and len(free)>0
    assert np.max(np.linalg.norm(poly[None]+free[:,None]-CENTER[:2],axis=2))<=RADIUS-.002+1e-9
def test_too_big_stays_rejected():
    free,fine=free_offsets_refined(circle(.0485,0,0),coarse_grid(),CENTER,RADIUS,OCC)
    assert fine and len(free)==0
def test_coarse_success_does_not_use_fine():
    free,fine=free_offsets_refined(circle(.03,0,0),coarse_grid(),CENTER,RADIUS,OCC)
    assert not fine and len(free)>0
