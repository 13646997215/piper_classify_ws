#!/usr/bin/env python3
"""Offline analysis of recorded RGB-D/TF pairs; never used by the detector."""
import argparse
import collections
import json
from pathlib import Path
import cv2
import numpy as np


def candidates(data):
    hsv = cv2.cvtColor(data['rgb'], cv2.COLOR_RGB2HSV)
    masks = {
        'red': cv2.inRange(hsv, (0,80,50), (12,255,255)) | cv2.inRange(hsv, (170,80,50), (179,255,255)),
        'green': cv2.inRange(hsv, (35,80,50), (90,255,255)),
        'blue': cv2.inRange(hsv, (95,80,50), (135,255,255)),
    }
    depth, k, matrix = data['depth'], data['k'], data['camera_to_base']
    v,u = np.indices(depth.shape)
    camera = np.stack([(u-k[0,2])*depth/k[0,0], (v-k[1,2])*depth/k[1,1], depth], axis=-1)
    points = camera @ matrix[:3,:3].T + matrix[:3,3]
    good = (np.isfinite(depth) & (depth>.05) & (depth<2) & (points[:,:,0]>.2)
            & (points[:,:,0]<.35) & (np.abs(points[:,:,1])<.2)
            & (points[:,:,2]>0) & (points[:,:,2]<.1))
    for color, mask in masks.items():
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
        contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        best = np.empty((0,3))
        for contour in contours:
            if cv2.contourArea(contour)<100: continue
            component = np.zeros(mask.shape,np.uint8)
            cv2.drawContours(component,[contour],0,255,cv2.FILLED)
            pts = points[good & (mask>0) & (component>0)]
            if len(pts)>len(best): best=pts
        if len(best)>=200:
            yield color,best


def main():
    parser=argparse.ArgumentParser();parser.add_argument('capture',type=Path)
    args=parser.parse_args()
    record=json.loads((args.capture/'record.json').read_text())
    errors=collections.defaultdict(list)
    rows=[]
    for sample in record['samples']:
        data=np.load(args.capture/sample['file'])
        for color,points in candidates(data):
            top=points[points[:,2]>=np.quantile(points[:,2],.75)-.004]
            if len(top)<100:continue
            old=np.median(top,axis=0);old[2]-=.025
            truth=np.array(record['truth_world'][color][:3]);truth[2]-=.347
            error=float(np.linalg.norm(old-truth)*1000)
            errors[sample['phase']].append(error)
            patch=points[np.abs(points[:,2]-.053)<.007]
            widths=(np.quantile(patch[:,:2],.99,axis=0)-np.quantile(patch[:,:2],.01,axis=0)).tolist() if len(patch)>10 else []
            rows.append({'phase':sample['phase'],'file':sample['file'],'color':color,
                         'synchronized_old_center':old.tolist(),'error_mm':error,
                         'top_height':float(np.median(top[:,2])),'top_patch_widths':widths})
    for phase,values in errors.items():
        print(phase,'n',len(values),'max_mm',round(max(values),2),'median_mm',round(float(np.median(values)),2))
    deltas=np.array([x['delta_ms'] for x in record['rgb_depth_deltas']])
    print('latest-depth deltas ms', 'p50,p90,p99,max', np.quantile(np.abs(deltas),[.5,.9,.99,1]).tolist())
    for row in sorted(rows,key=lambda r:r['error_mm'],reverse=True)[:10]:print(row)
    (args.capture/'analysis.json').write_text(json.dumps({'synchronized_old_estimates':rows,
        'latest_depth_abs_delta_ms_quantiles':np.quantile(np.abs(deltas),[.5,.9,.99,1]).tolist()},indent=2))


if __name__=='__main__':main()
