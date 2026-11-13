from pathlib import Path
import json,numpy as np
from mesh_geometry import read_stl,write_stl
P=Path(__file__).resolve().parents[1]
m=json.loads((P/'config/base_mesh_replacement.json').read_text())
holder=read_stl(P/m['preserved_holder']);base=read_stl(P/m['input'])/1000
base=base@np.array(m['CAD_to_base_rotation']).T+np.array(m['CAD_to_base_translation_m'])
a=np.concatenate([holder,base]);write_stl(P/'meshes/base_link_print.stl',a.reshape(-1,3),np.arange(a.size//3).reshape(-1,3))
print('Rebuilt base only; original motor holder retained.')
