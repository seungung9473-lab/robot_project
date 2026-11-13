"""Choose a validated dual-D415 transport profile; resolution stays 640x480."""
from pathlib import Path

def dual_camera_profile(requested='auto',root=Path('/sys/bus/usb/devices')):
    if requested in ('6','15'):return '640x480x'+requested,'explicit camera fps '+requested
    if requested!='auto':raise ValueError('Camera fps must be auto, 6 or 15')
    speeds=[]
    for node in Path(root).iterdir():
        try:
            if (node/'idVendor').read_text().strip()=='8086' and (node/'idProduct').read_text().strip()=='0ad3':speeds.append(float((node/'speed').read_text().strip()))
        except (OSError,ValueError):continue
    if len(speeds)<2:return '640x480x6','two D415 links not yet identified; conservative 6fps'
    if min(speeds)<5000:return '640x480x6','USB2 D415 detected; validated 6fps dual RGB-D'
    return '640x480x15','both D415 links are SuperSpeed; 15fps'
