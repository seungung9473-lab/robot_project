"""Decide whether the destination pad may be used, from the live pad observation.

- Anything standing on the pad that the cameras can see always stops the batch.
- The reserved slot must normally be seen (enough floor pixels). When the arm/turntable hide it,
  the start-of-batch free-slot check (which reserved this slot while the arm was at home and the view was clear)
  stands in; only our own placements enter a pad during a batch and those are tracked separately.
- Without that start-of-batch evidence an unseen pad still stops the batch.
"""
MIN_SURFACE_PIXELS=100
MAX_ELEVATED_PIXELS=40

def judge_pad(checks,start_of_batch_check):
    seen=[f"{cam}: floor {c['surface_pixels']} px, raised {c['elevated_pixels']} px" for cam,c in checks.items()]
    if any(c['elevated_pixels']>=MAX_ELEVATED_PIXELS for c in checks.values()):
        raise RuntimeError('Left pad obstructed: object seen on pad ('+'; '.join(seen)+')')
    if max(c['surface_pixels'] for c in checks.values())>=MIN_SURFACE_PIXELS:return 'observed'
    if start_of_batch_check:return 'start_of_batch_check'
    raise RuntimeError('Pad not observed sufficiently and no start-of-batch free-slot check ('+'; '.join(seen)+')')
