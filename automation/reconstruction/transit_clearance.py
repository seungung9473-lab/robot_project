"""Clearance for the empty-cup wait pose after a release.

The turntable top is z=59 mm and objects rise up to about z=99 mm, so a 30 mm wait above the pad (tip z=89 mm)
sits inside the objects' height and the next move/rotation can sweep them. Lift 100 mm (tip z about 159 mm) so the
cup and arm clear every object before the turntable rotates and before the next approach.
Planning (check_inclined_motion) and execution (verified_place_left) must use this same value.
"""
RETREAT_UP_M=0.10
