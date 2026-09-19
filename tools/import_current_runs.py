#!/usr/bin/env python3
"""Import frozen SITL layouts/results, without regenerating or running a scene."""
from __future__ import annotations
import argparse,json,math,xml.etree.ElementTree as ET
from pathlib import Path
import yaml


def pose(element):
    values=[float(x) for x in element.findtext('pose','0 0 0 0 0 0').split()]
    if len(values)!=6 or any(abs(v)>1e-8 for v in values[3:]):
        raise ValueError('Wall/model rotations require explicit geometry handling')
    return values


def import_case(run,seed,source):
    run=Path(run);inputs=run/'scenario_inputs'
    world=ET.parse(inputs/'field.world').getroot()
    field=next(m for m in world.iter('model') if m.get('name')=='toudi2')
    parent=pose(field);walls=[]
    for link in field.findall('link'):
        if not link.get('name','').startswith('Wall'):continue
        lp=pose(link)
        for collision in link.findall('collision'):
            node=collision.find('geometry/box/size')
            if node is None:raise ValueError('Non-box wall collision is not supported')
            size=[float(x) for x in node.text.split()];cp=pose(collision)
            centre=[parent[i]+lp[i]+cp[i] for i in range(3)]
            walls.append(dict(id=link.get('name'),x=centre[0],y=centre[1],size_x=size[0],size_y=size[1],height=centre[2]+size[2]/2))
    config=yaml.safe_load((inputs/'field_config.yaml').read_text())
    trees=[dict(id=t['name'],x=t['world_x'],y=t['world_y'],radius=t['radius']) for t in config['static_exclusions']]
    truth=yaml.safe_load((run/'random_field_truth.yaml').read_text())
    if any(abs(truth.get('spawn_offset',{}).get(k,0.))>1e-8 for k in ('x','y')):raise ValueError('Nonzero spawn offset needs an explicit route-frame transform')
    targets=[dict(class_name=t['class'],x=t['world_x'],y=t['world_y'],yaw=t.get('yaw',0.)) for t in truth['targets']]
    gate=json.loads((run/'gate_status.json').read_text());camera=json.loads((run/'actual_camera_info.json').read_text())
    manifest=yaml.safe_load((run/'manifest.yaml').read_text())
    if manifest.get('git_head')!=source:raise ValueError('Manifest and batch source differ')
    return dict(seed=seed,source=source,run=run.name,walls=walls,trees=trees,targets=targets,camera={k:camera[k] for k in ('width','height','K','D')},
                actual_gate=gate['status'],actual_reason=gate['reason'],actual_commits=gate['metrics']['release_commit_count'],
                actual_mission_ros_s=gate['metrics'].get('mission_ros_sec'),
                input_scope='Frozen world wall boxes + spawn exclusion footprints; trees are approximate, no truth enters flight control')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--matrix',type=Path,action='append',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    cases=[];seen=set();sources=[]
    for path in a.matrix:
        matrix=json.loads(path.read_text())
        if matrix['status']!='COMPLETE':raise ValueError('Only completed frozen batches may be imported')
        sources.append(dict(matrix=str(path),revision=matrix['source']))
        for row in matrix['results']:
            key=(row['seed'],matrix['source'],row['run'])
            if key in seen:raise ValueError('Duplicate input run')
            seen.add(key);case=import_case(row['run'],row['seed'],matrix['source'])
            case['actual_policy']='high_fast' if 'strategy:=true' in row.get('command',[]) else 'not_inferred'
            cases.append(case)
    result=dict(schema_version=1,scope='OFFLINE_FROZEN_SCENES_NOT_DETECTOR_OR_FLIGHT_GATE',sources=sources,cases=cases)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(cases=len(cases),seeds=[c['seed'] for c in cases],source_count=len(sources))))

if __name__=='__main__':main()
