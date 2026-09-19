#!/usr/bin/env python3
"""Current high-vs-low geometric screening. No flight/detector success prediction."""
from __future__ import annotations
import argparse,json,math
from dataclasses import replace
from pathlib import Path
import yaml
from search_sim import Point3,SearchArea,M0Parameters,generate_boustrophedon
from obstacles import Obstacle,plan_route_astar,route_collisions,line_of_sight_blockers
from recorded_search_replay import project_free,target_samples
from installed_camera import project,camera_origin,quaternion_from_rpy,rotation


def validate_profile(p):
    if p.get('schema_version')!=1 or p.get('guard_already_inflated') is not True:
        raise ValueError('Explicit already-inflated envelope is required')
    for name in ('high_agl_m','low_agl_m','resolution_m','max_endpoint_shift_m','high_speed_mps','low_speed_mps','turn_penalty_s','sample_hz','visibility_hold_s','tree_los_height_m','low_lane_spacing_m'):
        if not math.isfinite(p[name]) or p[name]<=0:raise ValueError('Invalid positive profile field: '+name)
    if not 0<p['low_agl_m']<p['high_agl_m']<=3.0:raise ValueError('Current profile altitude bounds')
    if not math.isfinite(p['camera_offset_z_m']) or p['low_agl_m']+p['camera_offset_z_m']<=0:raise ValueError('Camera must be above target plane')
    if p['sample_hz']>100 or p['max_endpoint_shift_m']>1.:raise ValueError('Sampling or endpoint search budget exceeded')
    if type(p['max_route_samples']) is not int or not 1<=p['max_route_samples']<=500000:raise ValueError('Invalid sample budget')
    if p['navigation_column_top_m']<p['high_agl_m']:raise ValueError('Obstacle overflight is prohibited')
    if not 0<=p['additional_clearance_m']<=.2:raise ValueError('Extra margin must be explicit and bounded')
    if len(p['guard_dimensions_m'])!=3 or any(not math.isfinite(v) or v<=0 for v in p['guard_dimensions_m']):raise ValueError('Invalid envelope')
    if len(p['body_rpy_deg'])!=3 or not all(math.isfinite(v) for v in p['body_rpy_deg']):raise ValueError('Invalid attitude')
    if len(set(p['required_classes']))!=3:raise ValueError('Exactly three distinct required classes')
    if p.get('detector_probability') is not None or p.get('flight_dynamics_or_gate_model') is not False:raise ValueError('No calibrated detector or flight model in this entry')
    b=p['planning_bounds_m'];s=p['search_bounds_m']
    if len(b)!=4 or len(s)!=4 or not all(math.isfinite(v) for v in b+s):raise ValueError('Invalid bounds')
    if not b[0]<=s[0]<s[1]<=b[1] or not b[2]<=s[2]<s[3]<=b[3]:raise ValueError('Search outside planning bounds')
    if (b[1]-b[0])*(b[3]-b[2])/p['resolution_m']**2>20000:raise ValueError('Grid exceeds resource budget')
    if not 2<=len(p['high_route_xy'])<=16 or any(len(q)!=2 or not all(math.isfinite(v) for v in q) for q in p['high_route_xy']):raise ValueError('Invalid high route')


def geometry(case,p):
    # Camera LOS sees the physical obstacle height. Navigation sees full columns.
    los=[Obstacle(w['id'],w['x'],w['y'],w['size_x'],w['size_y'],w['height']) for w in case['walls']]
    los += [Obstacle(t['id'],t['x'],t['y'],2*t['radius'],2*t['radius'],p['tree_los_height_m']) for t in case['trees']]
    nav=[replace(o,height=max(o.height,p['navigation_column_top_m'])) for o in los]
    q=quaternion_from_rpy(*[math.radians(v) for v in p['body_rpy_deg']]);r=rotation(q)
    half=[v/2 for v in p['guard_dimensions_m']]
    rx=sum(abs(r[0][i])*half[i] for i in range(3));ry=sum(abs(r[1][i])*half[i] for i in range(3))
    # Scalar square expansion conservatively bounds both horizontal projections.
    clearance=max(rx,ry)+p['additional_clearance_m']
    return los,nav,q,clearance


def planned_route(case,p,strategy):
    los,nav,q,clearance=geometry(case,p)
    if strategy=='high':raw=[Point3(x,y,p['high_agl_m']) for x,y in p['high_route_xy']]
    elif strategy=='low_coverage':
        s=p['search_bounds_m'];params=M0Parameters(SearchArea(*s),'x',p['low_lane_spacing_m'],p['low_agl_m'],p['low_speed_mps'],p['turn_penalty_s'])
        raw=[Point3(*p['high_route_xy'][0],p['low_agl_m'])]+generate_boustrophedon(params)
    else:raise ValueError('Unknown strategy')
    projected=[project_free(point,nav,p['planning_bounds_m'],p['resolution_m'],clearance,p['max_endpoint_shift_m']) for point in raw]
    if math.hypot(raw[0].x-projected[0].x,raw[0].y-projected[0].y)>1e-6:raise RuntimeError('Start cannot be silently relocated')
    unique=[projected[0]]
    for point in projected[1:]:
        if point!=unique[-1]:unique.append(point)
    if len(unique)<2:raise RuntimeError('No nonzero route')
    plan=plan_route_astar(unique,nav,tuple(p['planning_bounds_m']),p['resolution_m'],clearance)
    if route_collisions(plan.waypoints,nav,clearance):raise RuntimeError('Post-plan column check failed')
    return list(plan.waypoints),dict(clearance_m=clearance,extra_margin_m=p['additional_clearance_m'],max_endpoint_shift_m=max(math.hypot(a.x-b.x,a.y-b.y) for a,b in zip(raw,projected)),replanned_segments=plan.replanned_segment_count,expanded_nodes=plan.expanded_node_count)


def samples(route,speed,hz,turn_penalty):
    elapsed=0.;last=None
    for a,b in zip(route,route[1:]):
        length=math.hypot(b.x-a.x,b.y-a.y)
        if length<1e-9:continue
        direction=((b.x-a.x)/length,(b.y-a.y)/length)
        turning=last is not None and sum(x*y for x,y in zip(last,direction))<1-1e-8
        if turning:elapsed+=turn_penalty
        duration=length/speed;count=max(1,math.ceil(duration*hz));dt=duration/count
        for i in range(count):
            f=(i+1)/count
            yield elapsed+(i+1)*dt,Point3(a.x+(b.x-a.x)*f,a.y+(b.y-a.y)*f,a.z),dt,turning and i==0
        elapsed+=duration;last=direction


def evaluate(case,p,strategy,route):
    los,nav,q,clearance=geometry(case,p);required=p['required_classes']
    targets={t['class_name']:target_samples(dict(t,**{'class':t['class_name']})) for t in case['targets']}
    if len(targets)!=len(case['targets']):raise ValueError('Duplicate classes need instance-aware evaluation; refusing to collapse identities')
    if not set(required)<=set(targets):raise ValueError('Missing required target class')
    first={mode:{c:None for c in required} for mode in ('center','refine')};streak={(mode,c):0. for mode in first for c in required}
    stops={mode:None for mode in first};speed=p['high_speed_mps'] if strategy=='high' else p['low_speed_mps']
    count=sum(math.ceil(math.hypot(a.x-b.x,a.y-b.y)/speed*p['sample_hz']) for a,b in zip(route,route[1:]))
    if count>p['max_route_samples']:raise ValueError('Route sampling budget exceeded; change profile explicitly')
    for t,point,dt,turn in samples(route,speed,p['sample_hz'],p['turn_penalty_s']):
        if turn:streak={k:0. for k in streak}
        camera=Point3(*camera_origin(point,q,p['camera_offset_z_m']))
        for cls in required:
            points=targets[cls]
            in_frame=[project(v,point,case['camera'],q,p['camera_offset_z_m']) is not None for v in points]
            clear=[not line_of_sight_blockers(camera,v,los) for v in points]
            visible=dict(center=in_frame[0] and clear[0],refine=all(in_frame) and clear[0] and sum(clear)>=3)
            for mode,value in visible.items():
                key=mode,cls;streak[key]=streak[key]+dt if value else 0.
                if first[mode][cls] is None and streak[key]+1e-9>=p['visibility_hold_s']:first[mode][cls]=t
        for mode in first:
            if stops[mode] is None and all(v is not None for v in first[mode].values()):stops[mode]=dict(time_s=t,xy=[point.x,point.y])
    length=sum(math.hypot(a.x-b.x,a.y-b.y) for a,b in zip(route,route[1:]))
    directions=[((b.x-a.x),b.y-a.y) for a,b in zip(route,route[1:])]
    turns=sum(a[0]*b[0]+a[1]*b[1]<math.hypot(*a)*math.hypot(*b)-1e-8 for a,b in zip(directions,directions[1:]))
    return dict(route_distance_m=length,nominal_full_search_s=length/speed+turns*p['turn_penalty_s'],first_visibility_s=first,geometry_all_three=stops,
                missing={mode:[c for c,v in values.items() if v is None] for mode,values in first.items()},detector_probability=None,estimated_full_mission_s=None)


def compare(dataset,p):
    validate_profile(p);rows=[]
    if dataset.get('schema_version')!=1:raise ValueError('Unsupported frozen scene schema')
    for case in dataset['cases']:
        camera=case['camera']
        if camera['width']<=0 or camera['height']<=0 or len(camera['K'])!=9 or len(camera['D'])<5 or camera['K'][0]<=0 or camera['K'][4]<=0 or not all(math.isfinite(v) for v in camera['K']+camera['D']):
            raise ValueError('Invalid camera calibration; do not turn input errors into misses')
        for strategy in ('high','low_coverage'):
            row=dict(seed=case['seed'],source=case['source'],run=case['run'],strategy=strategy,actual_gate=case['actual_gate'],actual_policy=case.get('actual_policy','not_inferred'),actual_commits=case['actual_commits'],detector_probability=None,estimated_full_mission_s=None)
            try:
                route,meta=planned_route(case,p,strategy)
                row.update(status='GEOMETRY_FEASIBLE',route=[[v.x,v.y,v.z] for v in route],**meta,**evaluate(case,p,strategy,route))
            except RuntimeError as e:row.update(status='GEOMETRY_UNAVAILABLE',reason=str(e))
            rows.append(row)
    return dict(scope='KNOWN_MAP_GEOMETRY_ONLY_NOT_FLIGHT_GATE_OR_DETECTION',profile=p,guard_margin_counted_once=True,no_obstacle_overflight=True,
                note='Class/position truth is used only for offline viewing evaluation after route construction. Geometric visibility is not persistent detector confirmation. No low delivery/corridor/landing dynamics or task success model.',rows=rows)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--scenes',type=Path,required=True);ap.add_argument('--profile',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    result=compare(json.loads(a.scenes.read_text()),yaml.safe_load(a.profile.read_text()))
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2))
    print(json.dumps([dict(seed=r['seed'],strategy=r['strategy'],status=r['status'],all_three=r.get('geometry_all_three'),distance_m=r.get('route_distance_m')) for r in result['rows']],indent=2))

if __name__=='__main__':main()
