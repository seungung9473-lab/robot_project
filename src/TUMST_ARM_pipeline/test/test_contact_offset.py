import sys
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parents[3]/'automation/reconstruction'
sys.path.insert(0,str(R))
from contact_offset import right_direction,shifted_point,CONTACT_RIGHT_M
from inclined_geometry import patch

J1=np.array([1.4678,0.5220,0.16])

def test_right_is_to_the_right_of_the_base_to_point_direction():
    # 베이스에서 +Y(안쪽)로 뻗는 방향이면 오른쪽은 +X, −X(정면)로 뻗으면 오른쪽은 +Y
    assert np.allclose(right_direction(J1+[0,.3,0],J1),[1,0,0],atol=1e-9)
    assert np.allclose(right_direction(J1+[-.3,0,0],J1),[0,1,0],atol=1e-9)

def test_shift_is_five_mm_horizontal_and_stays_in_the_face_plane():
    point=J1+np.array([-.3,0,-.1]);n=np.array([0,0,1.])
    s=shifted_point(point,n,J1)
    assert abs(np.linalg.norm(s-point)-CONTACT_RIGHT_M)<1e-9 and abs(s[2]-point[2])<1e-12
    tilt=np.array([-.5,0,.8660254]);tilt/=np.linalg.norm(tilt);s2=shifted_point(point,tilt,J1)
    assert abs((s2-point)@tilt)<1e-12      # 기울어진 면이면 법선 성분은 빼서 면 위에 둔다

def flat_top(w=.06,h=.04,z=.06,step=.0015,seed=3):
    rng=np.random.default_rng(seed);xs=np.arange(-w/2,w/2,step);ys=np.arange(-h/2,h/2,step)
    g=np.array([(x,y,z+rng.normal(0,.0003)) for x in xs for y in ys]);return g+np.array([1.25,.5,0])

def test_flat_top_is_gripped_near_the_face_centre_not_at_an_arbitrary_edge():
    pts=flat_top();n=np.array([0,0,1.]);d=-.06
    best=patch(pts,n,d,prefer_high=True)
    centre=pts.mean(0);dist=np.linalg.norm(np.array(best['point_table_m'])[:2]-centre[:2])*1000
    assert best['geometry_pass'] and dist<4.0,dist

def test_planner_uses_the_offset_with_a_cup_fit_check():
    text=(R/'plan_inclined_pick.py').read_text()
    assert 'shifted_point(' in text and "contact_lateral_offset" in text and 'geometry_pass' in text.split('shifted_point(')[1][:900]
