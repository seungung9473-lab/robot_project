from pathlib import Path
import tempfile
import pytest
from soarm_bridge.camera_profile import dual_camera_profile

def select(speeds):
 with tempfile.TemporaryDirectory() as d:
  for i,speed in enumerate(speeds):
   p=Path(d)/str(i);p.mkdir()
   for name,value in [('idVendor','8086'),('idProduct','0ad3'),('speed',str(speed))]:(p/name).write_text(value)
  return dual_camera_profile(root=d)[0]

def test_usb2_uses_6fps():assert select([480,480])=='640x480x6'
def test_usb3_uses_15fps():assert select([5000,5000])=='640x480x15'
def test_mixed_link_uses_6fps():assert select([480,5000])=='640x480x6'
def test_missing_link_is_conservative():assert select([5000])=='640x480x6'
def test_explicit_profile():assert dual_camera_profile('15')[0]=='640x480x15'
def test_invalid_profile_rejected():
 with pytest.raises(ValueError):dual_camera_profile('30')
