"""Hold check between 'suction detected' and 'lift': the seal must stay below the held threshold for a short window.

Weak seals that only just cross the threshold (e.g. -106 / -156 on a tilted face) were declared successful and then
lost during the 40 mm lift. They now fall into the existing single 8 mm re-approach instead.
"""
SEAL_STABLE_S=1.0
MIN_SAMPLES=5   # the ESP32 reports ~10 Hz, so a 1 s window should hold ~10 fresh values

def evaluate(samples,held_diff,min_samples=MIN_SAMPLES):
    """samples: [(timestamp, diff)] collected during the window. Returns (stable, detail)."""
    values=[v for _,v in samples]
    if len(values)<min_samples:return False,{'samples':len(values),'reason':'too few fresh pressure samples'}
    worst=max(values)
    return worst<=held_diff,{'samples':len(values),'worst':worst,'best':min(values),'limit':held_diff}
