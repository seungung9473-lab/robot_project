"""Occupancy-aware joint reservation; planning only, no actuators."""
import json,sys
from pathlib import Path
from motion import kinematics
from tidy_pad_placement import reserve_pad
from test_data_recorder import start_trial
s=Path(sys.argv[1]);pad=int(sys.argv[2]);trial=start_trial('pad_reservation',physical=False,source=s)
routes=json.loads((s/'color_sort_plan.json').read_text())['objects']
keys=[row['object'] for row in routes if row.get('destination_pad_id')==pad]
path=reserve_pad(s,keys,pad,kinematics);trial.attach(path);trial.finish('planned');print(path,flush=True)
