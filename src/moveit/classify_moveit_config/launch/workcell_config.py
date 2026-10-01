"""Read fixed collision geometry from the same SDF world as Gazebo."""
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def parameters(description, world=None):
    result = yaml.safe_load((Path(description)/'config/workcell.yaml').read_text())
    root = ET.parse(world or Path(description)/'worlds/classify.world').getroot()
    # Counts are the task inventory, not pickup poses or visual identities.
    for colour in ['red', 'green', 'blue']:
        count=sum(bool(re.fullmatch(colour+r'_block(?:_[0-9]+)?', m.get('name','')))
                  for m in root.findall('world/model'))
        if count < 1 or count > 8: raise ValueError('Expected 1..8 workpieces per colour')
        if colour == 'green' and count != 1: raise ValueError('This task requires exactly one untouched green reference')
        result['expected_'+colour+'_count']=count
    def pose(element):
        x,y,z,r,p,w=map(float,element.findtext('pose','0 0 0 0 0 0').split())
        if abs(r)>1e-8 or abs(p)>1e-8:
            raise ValueError('Fixed workcell parser supports yaw-only box poses')
        return x,y,z,w
    def combine(a,b):
        x,y,z,w=a;dx,dy,dz,dw=b
        return x+math.cos(w)*dx-math.sin(w)*dy, y+math.sin(w)*dx+math.cos(w)*dy,z+dz,w+dw
    objects={'table_top':[],'table_legs':[],'bin_red':[],'bin_blue':[]}
    obstacles=[]
    for model in root.findall('world/model'):
        name=model.get('name')
        obstacle = name.startswith('obstacle_')
        if name not in ['table','bin_red','bin_blue'] and not obstacle:continue
        if obstacle:
            if model.findtext('static','false').strip().lower()!='true':
                raise ValueError('Planning obstacles must be fixed; dynamic objects require tracking')
            obstacles.append(name);objects[name]=[]
        for link in model.findall('link'):
            identity=('table_top' if link.get('name')=='table_top_link' else 'table_legs') if name=='table' else name
            lp=combine(pose(model),pose(link))
            for collision in link.findall('collision'):
                box=collision.find('geometry/box')
                if box is None:raise ValueError('Workcell collision must be a box')
                x,y,z,w=combine(lp,pose(collision));size=list(map(float,box.findtext('size').split()))
                objects[identity].extend([*size,x,y,z,w])
    if any(not boxes for boxes in objects.values()):raise ValueError('Missing fixed workcell geometry')
    result.update({name+'_boxes':boxes for name,boxes in objects.items()})
    if obstacles:result['obstacle_names']=obstacles
    return result
