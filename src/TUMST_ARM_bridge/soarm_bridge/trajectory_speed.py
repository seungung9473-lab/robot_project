"""Interpolate planned speed only; preserve the existing position path exactly."""
import math

def planned_speed(times, velocities, segment, fraction, fallback):
    # Missing or malformed velocity data keeps the former finite-difference behavior.
    if segment+1>=len(velocities):return fallback
    left,right=velocities[segment],velocities[segment+1]
    if left is None or right is None:return fallback
    if not math.isfinite(left) or not math.isfinite(right):return fallback
    if abs(fallback)<1e-12:return 0.0
    value=left+(right-left)*fraction
    # A speed command cannot reverse the position target. Ignore inconsistent signs.
    if value*fallback<0:return fallback
    return value
