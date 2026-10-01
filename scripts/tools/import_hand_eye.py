#!/usr/bin/env python3
"""Copy numeric calibration from toy_pick_ws; never execute or edit its code.
Usage: python3 scripts/tools/import_hand_eye.py [--source grasp_config.py]
Then regenerate/build URDF and run verify_camera_model.py.
The reference matrix maps RGB optical XYZ into link6, as used by visual_grasp.py.
"""
import argparse
import ast
import hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

root=Path(__file__).resolve().parents[2]
profile_path=root/'src/description/classify_description/config/gemini335.yaml'
parser=argparse.ArgumentParser()
parser.add_argument('--source',type=Path,default=Path.home()/'Desktop/toy_pick_ws/src/perception/toy_grasp/toy_grasp/grasp_config.py')
args=parser.parse_args()
data=args.source.read_bytes()
calibration=None
for item in ast.parse(data).body:
    if isinstance(item,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='HAND_EYE_CALIB' for t in item.targets):
        calibration=ast.literal_eval(item.value)
if calibration is None:raise ValueError('Missing literal HAND_EYE_CALIB dictionary')
profile=yaml.safe_load(profile_path.read_text())
he=profile['hand_eye']
xyz=np.asarray(calibration['position'],dtype=float)
rpy=np.asarray(calibration['rpy'],dtype=float)
if xyz.shape!=(3,) or rpy.shape!=(3,) or not np.isfinite(np.r_[xyz,rpy]).all():raise ValueError('Invalid calibration')
r=Rotation.from_euler('xyz',rpy).as_matrix()
body_to_optical=Rotation.from_euler('xyz',[-np.pi/2,0,-np.pi/2]).as_matrix()
offset=np.asarray(profile['depth']['color_origin_in_depth_optical_m'])
he.update(source=str(args.source.resolve()),source_sha256=hashlib.sha256(data).hexdigest(),
          xyz_m=xyz.tolist(),rpy_rad=rpy.tolist(),date=calibration['date'],samples=calibration['samples'],
          original_result_file=calibration['result_file'],
          original_result_available=(args.source.parent/calibration['result_file']).is_file(),
          body_xyz_m=(xyz-r@offset).tolist(),body_rpy_rad=Rotation.from_matrix(r@body_to_optical.T).as_euler('xyz').tolist())
profile_path.write_text(yaml.safe_dump(profile,sort_keys=False,allow_unicode=True))
print('Imported RGB optical calibration:',he['xyz_m'],he['rpy_rad'])
print('Regenerate URDF with bash scripts/tools/build_urdf.sh and verify the new model.')
