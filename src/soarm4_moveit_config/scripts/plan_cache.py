"""Cache policy: the robot pose may change, collision geometry may not."""
import hashlib,json,copy,math

def digest(data):return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def scene_digest(scene):
    data=copy.deepcopy(scene)
    robot=data.get('robot_state',{})
    robot.pop('joint_state',None);robot.pop('multi_dof_joint_state',None)
    def normalize(value):
        if isinstance(value,dict):return {k:normalize(v) for k,v in value.items() if k!='stamp'}
        if isinstance(value,list):return [normalize(v) for v in value]
        return value
    return digest(normalize(data))
def start_matches(names,trajectory,actual,tolerance=math.radians(1.0)):
    if len(names)!=len(actual) or not trajectory.points or len(trajectory.joint_names)!=len(names) or set(trajectory.joint_names)!=set(names):return False
    first=trajectory.points[0].positions
    if len(first)!=len(names):return False
    values=[first[trajectory.joint_names.index(n)] for n in names]
    return all(math.isfinite(a) and math.isfinite(b) and abs(a-b)<=tolerance for a,b in zip(actual,values))
