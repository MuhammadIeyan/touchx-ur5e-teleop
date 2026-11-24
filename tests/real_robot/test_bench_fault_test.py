import sys
import signal
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from real_robot.bench_fault_test import FaultStep


class FaultTests(unittest.TestCase):
    def state(self,kind):
        return SimpleNamespace(injected=False, inject_at=10., fault_kind=kind,
                               lease=SimpleNamespace(terminal=False,stop_requested=False),log=Mock())

    def test_drop_injection_only_after_deadline(self):
        obj=self.state('drop-touch')
        with patch('real_robot.bench_fault_test.time.monotonic',return_value=9.):
            FaultStep.inject_fault(obj)
        self.assertFalse(obj.injected)
        with patch('real_robot.bench_fault_test.time.monotonic',return_value=10.):
            FaultStep.inject_fault(obj)
        self.assertTrue(obj.injected)
        FaultStep.haptic(obj,object())  # Discarded: no freshness refresh, no base callback.
        self.assertFalse(hasattr(obj,'rx'))

    def test_sigterm_once_only_to_self(self):
        obj=self.state('sigterm')
        with patch('real_robot.bench_fault_test.time.monotonic',return_value=11.), \
             patch('real_robot.bench_fault_test.os.getpid',return_value=123), \
             patch('real_robot.bench_fault_test.os.kill') as kill:
            FaultStep.inject_fault(obj)
            FaultStep.inject_fault(obj)
            kill.assert_called_once_with(123,signal.SIGTERM)

    def test_no_injection_before_acceptance_or_after_stop(self):
        for field,value in [('terminal',True),('stop_requested',True)]:
            obj=self.state('sigterm');setattr(obj.lease,field,value)
            FaultStep.inject_fault(obj);self.assertFalse(obj.injected)
        obj=self.state('sigterm');obj.inject_at=None
        FaultStep.inject_fault(obj);self.assertFalse(obj.injected)


if __name__=='__main__':unittest.main()
