#!/usr/bin/env python3
"""Select a Gemini 335 USB stream mode using official V1.7 tables.
The hand-eye matrix and optical origins are preserved. Regenerate URDF afterward.
"""
import argparse
import math
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[2]
MODES={
    'rgb_4_3': dict(width=640,height=480,hfov=70.,vfov=55.,depth_focal=372.,min_z=.17),
    'rgb_16_9': dict(width=640,height=360,hfov=86.,vfov=55.,depth_focal=310.,min_z=.14),
}

def select(name, path):
    m=MODES[name];profile=yaml.safe_load(path.read_text())
    profile['stream_mode']=name
    profile['profile_source']='Official Gemini 330 USB datasheet V1.7, tables 4-4-1/4-5-2/4-5-3/MinZ'
    profile['datasheet_url']='https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini%20330%20series%20Datasheet-V1.7.pdf'
    profile['hardware_reference_streams']={'color':'640x480@15fps','depth':'640x480@15fps','note':'Recorded toy_pick_ws launch settings; current physical device stream has not been measured'}
    color,depth=profile['color'],profile['depth']
    color.update(width=m['width'],height=m['height'],fps=15,hfov_deg=m['hfov'],vfov_deg=m['vfov'],
        fx=m['width']/2/math.tan(math.radians(m['hfov']/2)),
        fy=m['height']/2/math.tan(math.radians(m['vfov']/2)),cx=m['width']/2,cy=m['height']/2)
    depth.update(width=m['width'],height=m['height'],fps=15,fx=m['depth_focal'],fy=m['depth_focal'],
        cx=m['width']/2,cy=m['height']/2,min_z_m=m['min_z'],
        intrinsics_source=f"Official typical {m['width']}x{m['height']} intrinsics; NOT unit-specific CameraInfo")
    path.write_text(yaml.safe_dump(profile,sort_keys=False,allow_unicode=True))
    print(f"Selected {name}: {m['width']}x{m['height']}@15; RGB {m['hfov']}x{m['vfov']} deg")
    print('Next: bash scripts/tools/build_urdf.sh; rebuild classify_description and restart Gazebo/MoveIt/perception')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=MODES)
    parser.add_argument('--profile',type=Path,default=ROOT/'src/description/classify_description/config/gemini335.yaml')
    args=parser.parse_args();select(args.mode,args.profile)
