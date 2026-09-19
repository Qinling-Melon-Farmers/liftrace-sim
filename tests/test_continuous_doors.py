import copy,importlib.util,json,tempfile,unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from r2026_scene import scene_layout,door_segments

spec=importlib.util.spec_from_file_location('exporter',Path(__file__).resolve().parents[1]/'tools/export_r2026_scene.py')
exporter=importlib.util.module_from_spec(spec);spec.loader.exec_module(exporter)

class ContinuousDoorTests(unittest.TestCase):
    def fixture(self,root):
        sdf=ET.Element('sdf',version='1.6');world=ET.SubElement(sdf,'world',name='default');model=ET.SubElement(world,'model',name='toudi2')
        for name,x,y,sx,sy in [('Wall_1',0,-.6,10,.2),('Wall_9',0,9.2,10,.2),('Wall_11',4.9,4.3,.2,9.6),('Wall_12',-4.9,4.3,.2,9.6),('Wall_13',.75,7.5,8.1,.2),('Wall_20_south',-1.6,7.95,.2,.7),('Wall_22_north',1.6,8.75,.2,.7)]:
            link=ET.SubElement(model,'link',name=name);ET.SubElement(link,'pose').text=f'{x} {y} .75 0 0 0'
            for kind in ('visual','collision'):
                box=ET.SubElement(ET.SubElement(ET.SubElement(link,kind,name=name+'_'+kind),'geometry'),'box');ET.SubElement(box,'size').text=f'{sx} {sy} 1.5'
        for i in range(4):ET.SubElement(ET.SubElement(model,'model',name=f'Tree_{i}'),'pose').text='0 0 0 0 0 0'
        ET.ElementTree(sdf).write(root/'template.world')
        (root/'field.yaml').write_text('{}')
        runtime=dict(mission=dict(post_delivery_route=[[0,8.35,.23] for _ in range(9)]),post_delivery_gate=dict(doors=[dict(name=n,lateral_min=0,lateral_max=0) for n in ('Wall_20','Wall_22')]))
        (root/'runtime.yaml').write_text(yaml.safe_dump(runtime))

    def test_continuous_distribution_independent_of_trees(self):
        centers=set()
        for seed in range(300):
            s=scene_layout(11,door_seed=seed,obstacle_seed=17,door_mode='continuous')
            self.assertEqual(s,scene_layout(11,door_seed=seed,obstacle_seed=17,door_mode='continuous'))
            self.assertEqual(s['trees'],scene_layout(11,door_seed=0,obstacle_seed=17)['trees'])
            for d in s['doors']:
                centers.add(round(d['center_y'],6));self.assertAlmostEqual(d['gap_max_y']-d['gap_min_y'],.8)
                self.assertAlmostEqual(sum(v['wall_length'] for v in door_segments(d)),.7)
        self.assertGreater(len(centers),590)

    def test_boundary_middle_and_tiny_slab_export(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.fixture(root)
            for index,center in enumerate((8.,8.35,8.7,8.000001)):
                layout=scene_layout(0,door_mode='continuous',door_centers=[center,center],outer_wall_height=4.)
                folder=root/str(index);exporter.export_scene(root/'template.world',root/'field.yaml',root/'runtime.yaml',folder,layout)
                field=ET.parse(folder/'field.world').find('.//model[@name="toudi2"]')
                for name in ('Wall_20','Wall_22'):
                    slabs=[l for l in field.findall('link') if l.get('name').startswith(name)]
                    self.assertEqual(len(slabs),1 if center in (8.,8.7) else 2)
                    intervals=[]
                    for slab in slabs:
                        y=float(slab.findtext('pose').split()[1]);size=list(map(float,slab.findtext('.//collision/geometry/box/size').split()))
                        self.assertGreater(size[1],0);self.assertEqual(size[2],1.5);intervals.append((y-size[1]/2,y+size[1]/2))
                    for lo,hi in intervals:self.assertTrue(hi<=center-.4+1e-9 or lo>=center+.4-1e-9)
                    self.assertAlmostEqual(sum(hi-lo for lo,hi in intervals)+.8,1.5)
                for name in ('Wall_1','Wall_9','Wall_11','Wall_12'):
                    link=field.find(f'link[@name="{name}"]');self.assertEqual(float(link.findtext('pose').split()[2]),2.)
                    for kind in ('collision','visual'):self.assertEqual(float(link.findtext(f'{kind}/geometry/box/size').split()[2]),4.)
                self.assertEqual(float(field.findtext('link[@name="Wall_13"]/collision/geometry/box/size').split()[2]),1.5)
                runtime=yaml.safe_load((folder/'experimental_runtime.yaml').read_text())
                self.assertNotIn('post_delivery_gate',runtime)
                self.assertTrue(all(point[1]==8.35 for point in runtime['mission']['post_delivery_route'][3:8]))

    def test_conflicts_invalid_width_range_and_height_reject(self):
        for args in [dict(door_mode='continuous',pattern='LR'),dict(door_mode='continuous',door_centers=[7.9,8.5]),dict(door_mode='continuous',door_centers=[8.1,float('nan')]),dict(outer_wall_height=4.1)]:
            with self.assertRaises(ValueError):scene_layout(1,**args)

if __name__=='__main__':unittest.main()
