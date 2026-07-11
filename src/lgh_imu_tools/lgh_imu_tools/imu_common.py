from __future__ import annotations
import math
from pathlib import Path
from typing import Iterable
import yaml
from ament_index_python.packages import get_package_share_directory

def default_contract_path() -> Path:
    return Path(get_package_share_directory('lgh_imu_tools'))/'config'/'imu_contract.yaml'

def load_contract(path: Path | None) -> dict:
    source=(path or default_contract_path()).expanduser().resolve()
    data=yaml.safe_load(source.read_text())
    if not isinstance(data,dict) or not isinstance(data.get('imu_contract'),dict):
        raise ValueError('imu_contract.yaml must contain imu_contract mapping')
    cfg=dict(data['imu_contract']); cfg['_path']=str(source)
    matrix=cfg.get('imu_to_base_matrix')
    if not isinstance(matrix,list) or len(matrix)!=9 or not all(math.isfinite(float(v)) for v in matrix):
        raise ValueError('imu_to_base_matrix must contain 9 finite values')
    return cfg

def finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(v)) for v in values)

def q_norm(q): return math.sqrt(sum(float(v)*float(v) for v in q))

def q_normalize(q):
    n=q_norm(q)
    if n<=1e-12: return (1.0,0.0,0.0,0.0)
    return tuple(float(v)/n for v in q)

def rotation_matrix_wxyz(q):
    w,x,y,z=q_normalize(q)
    return (
        1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
        2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
        2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y),
    )

def projected_gravity_sensor(q):
    # sensor_msgs/Imu orientation is treated as sensor->world. R^T * [0,0,-1].
    r=rotation_matrix_wxyz(q)
    return (-r[6],-r[7],-r[8])

def mat_vec(m,v):
    return (m[0]*v[0]+m[1]*v[1]+m[2]*v[2],m[3]*v[0]+m[4]*v[1]+m[5]*v[2],m[6]*v[0]+m[7]*v[1]+m[8]*v[2])

def euler_deg(q):
    w,x,y,z=q_normalize(q)
    roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    s=2*(w*y-z*x); pitch=math.asin(max(-1.0,min(1.0,s)))
    yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    return tuple(math.degrees(v) for v in (roll,pitch,yaw))

def percentile(values,p):
    if not values: return float('nan')
    s=sorted(values); x=(len(s)-1)*p; lo=int(math.floor(x)); hi=int(math.ceil(x))
    return s[lo] if lo==hi else s[lo]*(hi-x)+s[hi]*(x-lo)
