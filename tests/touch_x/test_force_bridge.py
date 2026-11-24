import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from geometry_msgs.msg import WrenchStamped
from touch_x_publisher import TouchPublisher

class ForceBridgeTests(unittest.TestCase):
    def bridge(self):
        return SimpleNamespace(failed=False, force_limit=0.5, force_stamp=-1,send_force=Mock(),
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10**9)),
            get_publishers_info_by_topic=lambda _: [SimpleNamespace(node_name='touch_x_sim_teleop',node_namespace='/')])
    def message(self):
        m=WrenchStamped();m.header.frame_id='touch_x_base';m.header.stamp.nanosec=950000000;m.wrench.force.y=.15;return m
    def test_configured_cap(self):
        b=self.bridge();b.force_limit=.8;m=self.message();m.wrench.force.y=.7
        TouchPublisher.on_force(b,m)
        self.assertAlmostEqual(b.send_force.call_args.args[0][1],.7)
        b.force_stamp=-1;m.wrench.force.y=.81
        TouchPublisher.on_force(b,m)
        self.assertEqual(b.send_force.call_args.args,([0.,0.,0.],0))

    def test_preserves_original_age(self):
        b=self.bridge();before=time.monotonic_ns();TouchPublisher.on_force(b,self.message());f,stamp=b.send_force.call_args.args
        self.assertAlmostEqual(f[1],.15)
        self.assertGreaterEqual(stamp,before-50_000_000)
        self.assertLessEqual(stamp,time.monotonic_ns()-50_000_000)
    def test_rejects_old_future_replayed_wrong_frame_large_nan_and_wrong_sender(self):
        for kind in ('old','future','replay','frame','large','nan','sender','torque'):
            b=self.bridge();m=self.message()
            if kind=='old':m.header.stamp.nanosec=800000000
            if kind=='future':m.header.stamp.sec=2
            if kind=='replay':b.force_stamp=950000000
            if kind=='frame':m.header.frame_id='world'
            if kind=='large':m.wrench.force.x=1.
            if kind=='nan':m.wrench.force.x=float('nan')
            if kind=='sender':b.get_publishers_info_by_topic=lambda _:[]
            if kind=='torque':m.wrench.torque.x=.01
            TouchPublisher.on_force(b,m)
            self.assertEqual(b.send_force.call_args.args,([0.,0.,0.],0),kind)
