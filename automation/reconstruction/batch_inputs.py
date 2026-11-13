"""Detect configuration/scan changes between full planning and execution."""
import hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def fingerprints(session):
 paths=[Path(session)/name for name in ['session.json','workcell_snapshot.yaml','inclined_pick_plan.json','color_sort_plan.json']]
 paths += [Path.home()/'robot_ws/src/TUMST_ARM_workcell/config/workcell.yaml',Path.home()/'robot_ws/src/TUMST_ARM_description/config/model.json',Path.home()/'robot_ws/src/TUMST_ARM_moveit_config/config/joint_limits.yaml']
 paths += [ROOT/name for name in ['batch_completion.py','continuous_pre_suction.py','normal_contact_trajectory.py','turntable_wait.py','verified_place_left.py','check_inclined_motion.py','execute_pick_batch.py','run_inclined_pick.py','wait_pose.py','trajectory_cache.py','transit_clearance.py','pad_clearance_policy.py','pad_labels.py','contact_offset.py','plan_inclined_pick.py']]
 paths += [ROOT/name for name in ['expiry_policy.py','expiry_sort_policy.json','trash_destination.py','batch_search.py','planning_feedback.py','pad_occupancy.py','tidy_pad_placement.py','pad_packing.py','plan_batch_pad.py','plan_pick_batch.py','read_numeric_dates.py','read_expiry_labels.py','payload_placement.py','date_crops.py','identity_routing.py','apply_sam2.py','sam2_shared.py','shared_observations.py','inclined_geometry.py','plan_inclined_pick.py','process_three_color_objects.py','reconstruct.py']]
 paths += [p for p in [Path.home()/'robot_ws/src/TUMST_ARM_workcell/config/named_poses.yaml',Path(session)/'expiry_route_report.json',Path(session)/'expiry_observations.json'] if p.exists()]
 paths += list(Path(session).glob('objects/*/analysis/cloud.npz'))
 paths += [p for p in [Path(session)/'pad_occupancy.json'] if p.exists()]
 return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
