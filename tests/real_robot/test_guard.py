import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.guard import Heartbeat,Guard,CONTROLLER

class GuardTests(unittest.TestCase):
    def test_source_time_not_arrival_time_controls_expiry(self):
        h=Heartbeat();h.feed(b'A 10.0\nH 10.1\n',10.11)
        self.assertFalse(h.expired(10.3))
        h.feed(b'H 10.1\n',10.4)
        self.assertTrue(h.expired(10.4))

    def test_fragmented_heartbeat_and_invalid_values(self):
        h=Heartbeat();h.feed(b'A 1',1.1);self.assertFalse(h.armed)
        h.feed(b'.0\n',1.1);self.assertTrue(h.armed)
        for data in (b'H nan\n',b'H 5\n',b'X 1.1\n'):
            with self.assertRaises(ValueError):h.feed(data,1.2)

    def test_guard_kills_unresponsive_child_before_deactivation(self):
        child=Mock();child.poll.return_value=None
        obj=SimpleNamespace(stopping=False,child=child,heartbeat=SimpleNamespace(armed=True),log=Mock())
        Guard.stop(obj,'expired')
        self.assertTrue(obj.stopping);child.kill.assert_called_once()
        Guard.stop(obj,'again');child.kill.assert_called_once()

    def test_active_controller_gets_deactivation_request(self):
        future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(controller=[SimpleNamespace(name=CONTROLLER,state='active')]))
        switch=SimpleNamespace(service_is_ready=lambda:True,call_async=Mock())
        obj=SimpleNamespace(done=False,stopping=True,deadline=float('inf'),switch_request=None,request=future,
                            switch_client=switch,log=Mock())
        Guard.tick(obj)
        request=switch.call_async.call_args.args[0]
        self.assertEqual(request.deactivate_controllers,[CONTROLLER])
        self.assertEqual(request.activate_controllers,[])

    def test_inactive_inventory_is_required_to_confirm_stop(self):
        future=SimpleNamespace(done=lambda:True,result=lambda:SimpleNamespace(controller=[SimpleNamespace(name=CONTROLLER,state='inactive')]))
        obj=SimpleNamespace(done=False,confirmed=False,stopping=True,deadline=float('inf'),switch_request=None,request=future,
                            child=SimpleNamespace(poll=lambda:-9),log=Mock())
        Guard.tick(obj)
        self.assertTrue(obj.done);self.assertTrue(obj.confirmed)

if __name__=='__main__':unittest.main()
