#!/usr/bin/env python3
"""Generate a reproducible harder world without changing the accepted baseline."""
import argparse
import copy
import math
import hashlib
import importlib.util
import json
from pathlib import Path
import random
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]


def box(world, name, pose, size):
    model=ET.SubElement(world,'model',name=name)
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=' '.join(map(str,pose))
    link=ET.SubElement(model,'link',name='body')
    for kind in ['collision','visual']:
        item=ET.SubElement(link,kind,name=kind)
        geometry=ET.SubElement(item,'geometry')
        ET.SubElement(ET.SubElement(geometry,'box'),'size').text=' '.join(map(str,size))
        if kind=='visual':
            material=ET.SubElement(item,'material')
            ET.SubElement(material,'ambient').text='.48 .48 .48 1'
            ET.SubElement(material,'diffuse').text='.48 .48 .48 1'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--profile',choices=['jitter','clutter','multi'],default='clutter')
    parser.add_argument('--seed',type=int,default=7)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Output already exists; choose a new path to preserve evidence')
    source=ROOT/'src/description/classify_description/worlds/classify.world'
    tree=ET.parse(source);world=tree.getroot().find('world');rng=random.Random(args.seed)
    poses={}
    for name in ['red_block','blue_block']:
        element=world.find(f"model[@name='{name}']/pose")
        pose=list(map(float,element.text.split()))
        # Small bounded position variation; arbitrary yaw needs oriented grasping.
        pose[0]+=rng.uniform(-.018,0.)
        pose[1]+=rng.uniform(-.018,.018)
        element.text=' '.join(map(str,pose));poses[name]=pose
    if args.profile=='multi':
        # Separate same-colour cubes in image and 3D; green stays unchanged.
        placements={'red_block':(.50,.073,22), 'blue_block':(.50,-.065,-18),
                    'red_block_2':(.40,0.,12), 'blue_block_2':(.40,-.14,-10)}
        for name,(x,y,degrees) in placements.items():
            model=world.find(f"model[@name='{name}']")
            if model is None:
                model=copy.deepcopy(world.find(f"model[@name='{name.split('_')[0]}_block']"))
                model.set('name',name);world.append(model)
            pose=[x+rng.uniform(-.001,.001),y+rng.uniform(-.001,.001),.375,0,0,math.radians(degrees+rng.uniform(-1,1))]
            model.find('pose').text=' '.join(map(str,pose));poses[name]=pose
        # Two-slot bins need a wider cavity; keep walls/floor identical in
        # Gazebo and MoveIt by editing this one SDF, never hardcoding boxes.
        for c,sign in [('red',-1),('blue',1)]:
            model=world.find(f"model[@name='bin_{c}']")
            model.find('pose').text=f'.52 {sign*.27} .35 0 0 {-math.pi/2}'
            for item in list(model.find('link')):
                if item.tag not in ['collision','visual']:continue
                size=list(map(float,item.findtext('geometry/box/size').split()))
                size=[.20 if abs(v-.13)<1e-6 else v for v in size]
                item.find('geometry/box/size').text=' '.join(map(str,size))
                pose=list(map(float,item.findtext('pose').split()))
                pose=[math.copysign(.094,v) if abs(abs(v)-.059)<1e-6 else v for v in pose]
                item.find('pose').text=' '.join(map(str,pose))
    obstacles=[]
    if args.profile=='clutter':
        for name,pose,size in [
            ('obstacle_between_bins',[.53,0.,.39,0.,0.,0.],[.055,.08,.08]),
            ('obstacle_front_corner',[.25,-.31,.39,0.,0.,0.],[.065,.065,.08]),
        ]:
            box(world,name,pose,size);obstacles.append({'name':name,'pose':pose,'size':size})
        world.find("light[@name='sun']/diffuse").text='.65 .65 .65 1'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(tree,space='  ');tree.write(args.output,encoding='utf-8',xml_declaration=True)
    # Check the exact world also feeds MoveIt, including both obstacle boxes.
    module_path=ROOT/'src/moveit/classify_moveit_config/launch/workcell_config.py'
    spec=importlib.util.spec_from_file_location('scenario_geometry',module_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    geometry=module.parameters(ROOT/'src/description/classify_description',args.output)
    assert all(o['name']+'_boxes' in geometry for o in obstacles)
    metadata={'profile':args.profile,'seed':args.seed,'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
              'world':str(args.output.resolve()),'world_sha256':hashlib.sha256(args.output.read_bytes()).hexdigest(),
              'pickup_poses':poses,'obstacles':obstacles,'green_unchanged':True,
              'limits':('Two red/two blue 50 mm cubes; oriented tops and two slots per bin' if args.profile=='multi' else 'One 50 mm cube per colour; yaw unchanged')+'; physics constraint grasp remains ideal'}
    args.output.with_suffix('.json').write_text(json.dumps(metadata,indent=2))
    print('Scenario:',args.output.resolve())
    print('Additional fixed obstacles:',len(obstacles))


if __name__=='__main__':main()
