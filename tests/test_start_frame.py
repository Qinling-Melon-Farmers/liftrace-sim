import math
import unittest
from tools.rotate_start_frame import xy,bounds,rotate_gate,rotate_runtime

class StartFrameTest(unittest.TestCase):
    def test_basis_and_roundtrip(self):
        self.assertEqual(xy([0,1,3]),[1,0,3])
        self.assertEqual(xy([-1,0]),[0,1])
        p=[2,3,4]
        for _ in range(4):p=xy(p)
        self.assertEqual(p,[2,3,4]);self.assertEqual(bounds([-4.8,4.8,-.5,9.1]),[-.5,9.1,-4.8,4.8])
    def test_gate_direction_and_clear_width(self):
        data={'post_delivery_gate':{'low_height_region':dict(min_x=-5,max_x=5,min_y=7,max_y=9),'doors':[dict(axis='x',coordinate=-1.6,direction='positive',lateral_min=8,lateral_max=8.8),dict(axis='y',coordinate=7.5,direction='positive',lateral_min=-4.8,lateral_max=-3.3)]}}
        gate=rotate_gate(data)['post_delivery_gate'];a,b=gate['doors']
        self.assertEqual((a['axis'],a['coordinate'],a['direction']),('y',1.6,'negative'))
        self.assertAlmostEqual(a['lateral_max']-a['lateral_min'],.8)
        self.assertEqual((b['lateral_min'],b['lateral_max']),(3.3,4.8))
        self.assertEqual(data['post_delivery_gate']['doors'][0]['axis'],'x')

if __name__=='__main__':unittest.main()
