#!/usr/bin/env python3
"""Explicit commissioning fault injection; unchanged single-step travel limits.

drop-touch discards received Touch messages after acceptance, exercising the real
freshness watchdog. sigterm sends SIGTERM to this process, exercising orderly
shutdown. Neither test claims to simulate SIGKILL or a robot network failure.
"""
import argparse
import os
import signal
import time

from real_robot.bench_step import BenchStep, main as run_step


class FaultStep(BenchStep):
    fault_kind = None

    def __init__(self, *args, **kwargs):
        self.inject_at = None
        self.injected = False
        super().__init__(*args, **kwargs)
        self.create_timer(.01, self.inject_fault)
        self.log('fault_test_configuration', kind=self.fault_kind, delay_after_acceptance_s=.5)

    def accepted(self, future):
        super().accepted(future)
        if self.lease.handle is not None and not self.lease.stop_requested:
            self.inject_at = time.monotonic() + .5

    def inject_fault(self):
        if (self.injected or self.inject_at is None or self.lease.terminal
                or self.lease.stop_requested or time.monotonic() < self.inject_at):
            return
        self.injected = True
        self.log('fault_injected', kind=self.fault_kind)
        if self.fault_kind in ('sigterm', 'sigkill'):
            os.kill(os.getpid(), signal.SIGTERM if self.fault_kind == 'sigterm' else signal.SIGKILL)

    def haptic(self, msg):
        if self.injected and self.fault_kind == 'drop-touch':
            return
        super().haptic(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fault', choices=('drop-touch', 'sigterm', 'sigkill'), required=True)
    parser.add_argument('--session', required=True)
    args = parser.parse_args()
    if args.fault == 'sigkill' and 'UR5E_GUARD_FD' not in os.environ:
        parser.error('SIGKILL trial requires the independent guard launcher')
    FaultStep.fault_kind = args.fault
    run_step(['--execute-one-step', '--release-test', '--session', args.session], node_type=FaultStep)


if __name__ == '__main__':
    main()
