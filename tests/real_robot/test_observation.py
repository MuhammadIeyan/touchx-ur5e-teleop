import ast,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.observe import Observer
from real_robot.report import summarize
from geometry_msgs.msg import PoseStamped
class ObservationTests(unittest.TestCase):
    def test_no_command_capability(self):
        tree=ast.parse((Path(__file__).resolve().parents[2]/'scripts/real_robot/observe.py').read_text())
        calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertNotIn('create_publisher',calls)
        # The only client is the read-only controller inventory service.
        clients=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_client']
        self.assertEqual(len(clients),1)
        self.assertEqual(clients[0].args[0].id,'ListControllers')
    def test_tcp_invalid_and_replayed(self):
        obj=SimpleNamespace(tcp=None,last_tcp_stamp=-1,stamp=lambda h:100)
        msg=PoseStamped();msg.header.frame_id='base';msg.pose.orientation.w=1.;msg.pose.position.x=.4
        Observer.on_tcp(obj,msg);self.assertEqual(obj.tcp[0],.4)
        Observer.on_tcp(obj,msg);self.assertIsNone(obj.tcp)
        obj.last_tcp_stamp=-1;msg.header.frame_id='world';Observer.on_tcp(obj,msg);self.assertIsNone(obj.tcp)
        msg.header.frame_id='base';msg.pose.position.x=float('nan');Observer.on_tcp(obj,msg);self.assertIsNone(obj.tcp)
    def test_report_does_not_claim_preview_accuracy(self):
        rows=[dict(mode='preview_only',monotonic_s=i*.02,reason='ready',tcp=[i*.001,0,0,0,0,0,1],preview_xyz=[1,0,0]) for i in range(3)]
        result=summarize(rows)
        self.assertFalse(result['accuracy_assessed'])
        np.testing.assert_allclose(result['tcp_span_mm'],[2,0,0])
