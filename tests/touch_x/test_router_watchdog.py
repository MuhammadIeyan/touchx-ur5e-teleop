import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from command_router import UR5eCommandRouter


class RouterWatchdogTests(unittest.TestCase):
    def router(self):
        now=time.monotonic()
        return SimpleNamespace(actual_positions=[0.]*6, joint_time=now,
            require_deadman=True,deadman=True,deadman_time=now-1,
            output_positions=[1.]*6,target_positions=[2.]*6,
            last_update_time=now-.01,last_command_time=now,
            command_received=True,timeout_active=False,simulation_domain=True,
            robot_pub=Mock(),get_logger=lambda:Mock())

    def test_deadman_timeout_freezes_output_not_pending_target(self):
        r=self.router();UR5eCommandRouter.update(r)
        self.assertEqual(r.output_positions,[1.]*6)
        self.assertEqual(list(r.robot_pub.publish.call_args[0][0].data),[1.]*6)

    def test_no_commands_before_first_input(self):
        r=self.router();r.command_received=False;UR5eCommandRouter.update(r)
        r.robot_pub.publish.assert_not_called()

    def test_stale_simulation_state_no_publish(self):
        r=self.router();r.joint_time-=1;UR5eCommandRouter.update(r)
        r.robot_pub.publish.assert_not_called()

if __name__=='__main__':unittest.main()
