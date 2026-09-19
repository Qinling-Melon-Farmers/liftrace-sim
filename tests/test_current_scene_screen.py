import copy,math,unittest
from pathlib import Path
import yaml
from current_scene_screen import geometry,planned_route,evaluate,validate_profile,samples,compare
from obstacles import route_collisions
from search_sim import Point3

class CurrentScreenTests(unittest.TestCase):
    def setUp(self):
        self.p=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/high_view_20260919.yaml').read_text())
        self.p.update(planning_bounds_m=[-2.,2.,-2.,2.],search_bounds_m=[-1.5,1.5,-1.5,1.5],high_route_xy=[[-1.5,0.],[1.5,0.]])
        self.c=dict(seed=1,source='fixture',run='fixture',actual_gate='FAIL',actual_commits=0,walls=[],trees=[dict(id='tree',x=0.,y=0.,radius=.3)],
                    camera=dict(K=[725,0,640,0,725,360,0,0,1],D=[0,0,0,0,0],width=1280,height=720),
                    targets=[dict(class_name=n,x=x,y=.8,yaw=0.) for n,x in zip(self.p['required_classes'],[-1.,0.,1.])])

    def test_guard_margin_is_counted_once(self):
        _,_,_,margin=geometry(self.c,self.p)
        self.assertAlmostEqual(margin,.275)
        self.p['additional_clearance_m']=.02
        self.assertAlmostEqual(geometry(self.c,self.p)[3],.295)

    def test_yaw_projection_includes_both_guard_axes(self):
        self.p['body_rpy_deg']=[0.,0.,45.]
        self.assertAlmostEqual(geometry(self.c,self.p)[3],.275*math.sqrt(2))

    def test_high_navigation_cannot_cross_tree_but_los_keeps_real_height(self):
        los,nav,_,margin=geometry(self.c,self.p)
        straight=[Point3(-1.5,0.,2.6),Point3(1.5,0.,2.6)]
        self.assertFalse(route_collisions(straight,los,margin))
        self.assertTrue(route_collisions(straight,nav,margin))
        route,meta=planned_route(self.c,self.p,'high')
        self.assertFalse(route_collisions(route,nav,margin))
        self.assertGreater(meta['replanned_segments'],0)
        self.assertGreater(sum(math.hypot(a.x-b.x,a.y-b.y) for a,b in zip(route,route[1:])),3.)

    def test_route_builder_does_not_consume_target_coordinates(self):
        route,_=planned_route(self.c,self.p,'high');other=copy.deepcopy(self.c)
        for t in other['targets']:t.update(x=100.,y=-100.)
        self.assertEqual(route,planned_route(other,self.p,'high')[0])

    def test_no_free_start_is_not_silently_moved(self):
        self.p['high_route_xy'][0]=[0.,0.]
        with self.assertRaises(RuntimeError):planned_route(self.c,self.p,'high')

    def test_infeasible_wall_route_raises(self):
        self.c['walls']=[dict(id='wall',x=0.,y=0.,size_x=.2,size_y=8.,height=1.5)]
        with self.assertRaises(RuntimeError):planned_route(self.c,self.p,'high')

    def test_visibility_is_not_detection_probability_or_full_mission_time(self):
        self.c['trees']=[]
        route,_=planned_route(self.c,self.p,'high');v=evaluate(self.c,self.p,'high',route)
        self.assertIsNone(v['detector_probability']);self.assertIsNone(v['estimated_full_mission_s'])
        self.assertIsNotNone(v['geometry_all_three']['center'])
        self.assertGreaterEqual(v['geometry_all_three']['center']['time_s'],self.p['visibility_hold_s'])

    def test_duplicate_classes_are_not_silently_merged(self):
        self.c['targets'].append(dict(self.c['targets'][0]))
        route,_=planned_route(self.c,self.p,'high')
        with self.assertRaises(ValueError):evaluate(self.c,self.p,'high',route)

    def test_required_three_not_first_three_any_classes(self):
        self.c['trees']=[];self.c['targets'][-1].update(x=50.,y=50.)
        self.c['targets'].append(dict(class_name='tent',x=0.,y=0.,yaw=0.))
        route,_=planned_route(self.c,self.p,'high');v=evaluate(self.c,self.p,'high',route)
        self.assertIsNone(v['geometry_all_three']['center'])
        self.assertIn('red_cross',v['missing']['center'])

    def test_invalid_policy_cannot_enable_overflight_or_probabilities(self):
        for key,value in [('navigation_column_top_m',1.8),('detector_probability',.9),('guard_already_inflated',False),('resolution_m',.0001),('high_speed_mps',0.),('camera_offset_z_m',-2.)]:
            p=copy.deepcopy(self.p);p[key]=value
            with self.assertRaises(ValueError):validate_profile(p)

    def test_short_segments_are_sampled_and_end_time_is_causal(self):
        points=list(samples([Point3(0,0,2.6),Point3(.01,0,2.6)],1.,10.,1.))
        self.assertEqual(len(points),1);self.assertAlmostEqual(points[0][0],.01)
        self.assertAlmostEqual(points[0][1].x,.01)

    def test_bad_inputs_cannot_be_reported_as_geometric_misses(self):
        with self.assertRaises(ValueError):compare(dict(schema_version=2,cases=[self.c]),self.p)
        self.c['camera']['K'][0]=0.
        with self.assertRaises(ValueError):compare(dict(schema_version=1,cases=[self.c]),self.p)

    def test_sample_budget_bounds_very_slow_profiles(self):
        self.p['high_speed_mps']=.00001
        route,_=planned_route(self.c,self.p,'high')
        with self.assertRaises(ValueError):evaluate(self.c,self.p,'high',route)

if __name__=='__main__':unittest.main()
