from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path('/home/hanhan/Desktop/piper_classify_ws')
PKG = ROOT / 'src/description/classify_description'
urdf = ET.parse(PKG / 'urdf/piper_sim.urdf').getroot()
world = ET.parse(PKG / 'worlds/classify.world').getroot()
joints = {j.get('name'): j for j in urdf.findall('joint')}
q_home = yaml.safe_load((PKG / 'config/gazebo_initial_positions.yaml').read_text())['initial_positions']

def transform(j):
    e = j.find('origin')
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler('xyz', list(map(float, e.get('rpy', '0 0 0').split()))).as_matrix()
    t[:3, 3] = list(map(float, e.get('xyz', '0 0 0').split()))
    return t

chain = [joints[f'joint{i}'] for i in range(1, 7)]
origins = [transform(j) for j in chain]
axes = [np.array(list(map(float, j.find('axis').get('xyz').split()))) for j in chain]
lo = np.array([float(j.find('limit').get('lower')) for j in chain])
hi = np.array([float(j.find('limit').get('upper')) for j in chain])
home = np.array([q_home[f'joint{i}'] for i in range(1, 7)])
base_world = np.eye(4); base_world[2, 3] = 0.347

def fk(q):
    t = np.eye(4)
    for origin, axis, qi in zip(origins, axes, q):
        m = np.eye(4); m[:3, :3] = Rotation.from_rotvec(axis * qi).as_matrix()
        t = t @ origin @ m
    return t

def tcp(q):
    t = fk(q).copy(); t[:3, 3] += t[:3, :3] @ [0, 0, 0.135]
    return t

def desired(pos, tool_z):
    z = np.asarray(tool_z, float); z /= np.linalg.norm(z)
    up = np.array([0, 0, 1.0]) if abs(z[2]) < 0.98 else np.array([0, 1.0, 0])
    x = np.cross(up, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    t = np.eye(4); t[:3, :3] = np.column_stack([x, y, z]); t[:3, 3] = pos
    return t

def residual(q, target):
    t = tcp(q)
    return np.r_[t[:3, 3] - target[:3, 3], Rotation.from_matrix(target[:3, :3].T @ t[:3, :3]).as_rotvec()]

def solve(target, seeds=16):
    rng = np.random.default_rng(42); best = None
    for i in range(seeds):
        seed = np.clip(home, lo + 1e-5, hi - 1e-5) if i == 0 else rng.uniform(lo, hi)
        s = least_squares(residual, seed, args=(target,), bounds=(lo, hi), max_nfev=300,
                          xtol=1e-10, ftol=1e-10, gtol=1e-10)
        if best is None or s.cost < best.cost: best = s
        e = residual(s.x, target)
        if np.linalg.norm(e[:3]) < 1e-6 and np.linalg.norm(e[3:]) < 1e-6: break
    e = residual(best.x, target)
    return {'position_mm': float(np.linalg.norm(e[:3])*1000),
            'rotation_deg': float(np.linalg.norm(e[3:])*180/np.pi),
            'joints_deg': np.degrees(best.x).tolist()}

results = {'limits': {'lower': lo.tolist(), 'upper': hi.tolist()}, 'home': {}, 'camera': {}, 'old_ik': [], 'corrected_ik': []}
h = tcp(home); c = base_world @ fk(home) @ transform(joints['link6_to_camera'])
results['home'] = {'tcp_base': h[:3, 3].tolist(), 'tool_z_world': h[:3, 2].tolist(),
                   'limit_violations': [f'joint{i+1}' for i,v in enumerate(home) if v < lo[i] or v > hi[i]]}
results['camera'] = {'position_world': c[:3, 3].tolist(), 'gazebo_forward_world': c[:3, 0].tolist()}
print('HOME', results['home'], flush=True); print('CAMERA', results['camera'], flush=True)
for name in ['red_block', 'green_block', 'blue_block']:
    model = world.find(f"world/model[@name='{name}']")
    p = np.array(list(map(float, model.findtext('pose').split()[:3])))
    ps = (np.linalg.inv(c) @ np.r_[p, 1])[:3]
    optical = np.array([-ps[1], -ps[2], ps[0]])
    fx = 640/(2*np.tan(1.047/2))
    uv = np.array([fx*optical[0]/optical[2]+320, fx*optical[1]/optical[2]+240])
    print('HOME_CAMERA_PROJECTION', name, 'optical', optical.round(5), 'uv', uv.round(2), 'in_frustum', bool(optical[2]>0 and 0<=uv[0]<640 and 0<=uv[1]<480), flush=True)

# Load definitions from the existing checker without executing its reporting loop.
old = {}; code = (ROOT/'scripts/tools/check_ik.py').read_text().split('print("=== IK')[0]
exec(compile(code, 'old_checker_definitions', 'exec'), old)
for name, x, y, z, up in old['WORK']:
    target = old['target'](x, y, z + up, old['TOOL_Z'])
    s = old['ik'](target, seeds=12)
    actual_angle = np.linalg.norm(Rotation.from_matrix(target[:3,:3].T @ fk(s.x)[:3,:3]).as_rotvec())*180/np.pi
    actual_world_tcp = (base_world @ np.r_[tcp(s.x)[:3,3], 1])[:3]
    entry = {'name': name, 'checker_rot_residual_deg': float(np.linalg.norm(old['err'](s.x,target)[3:])*180/np.pi),
             'actual_rotation_deg': float(actual_angle), 'tcp_world': actual_world_tcp.tolist(),
             'target_world': [x,y,z+up]}
    results['old_ik'].append(entry); print('OLD_IK', entry, flush=True)
for direction_name, direction in [('tilted45', [0, 1, -1]), ('vertical', [0, 0, -1])]:
    for name, x, y, z, up in old['WORK']:
        target = desired([x,y,z+up-0.347], direction)
        entry = {'direction': direction_name, 'name': name, **solve(target)}
        results['corrected_ik'].append(entry); print('CORRECTED_IK', entry, flush=True)
Path('/tmp/piper_classify_audit/kinematics_results.json').write_text(json.dumps(results, indent=2))
