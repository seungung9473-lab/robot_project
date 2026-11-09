"""Depth-edge guard for reconstruction masks.

SAM2 masks are tight in 2D, but depth at object boundaries bleeds toward the background, which fattens the 3D
footprint (cam1 flat box: r=50 mm vs real 32 mm). Depth points use the mask eroded by ERODE_PX; the footprint
used for pad placement gets FOOTPRINT_MARGIN_M back (pad_occupancy.inflate_polygon). Color/QR reading is untouched.
"""
import numpy as np

ERODE_PX=3
MIN_KEEP_FRACTION=.25

def erode_depth_mask(mask,px=ERODE_PX,min_keep=MIN_KEEP_FRACTION):
    """Return the mask eroded by px (square kernel). If too little remains (small/thin object), keep the original."""
    import cv2
    m=np.asarray(mask,bool)
    if px<=0 or not m.any():return m
    eroded=cv2.erode(m.astype(np.uint8),np.ones((2*px+1,2*px+1),np.uint8))>0
    return eroded if eroded.sum()>=min_keep*m.sum() else m
