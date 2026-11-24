"""Independent process watchdog for bounded bench commands, domain 43 only.

On child exit or missed heartbeat: kill the child to prevent later resumption,
then deactivate its motion controller. This is not a safety-rated controller.
Each accepted robot goal must still have independently bounded travel.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from controller_manager_msgs.srv import ListControllers, SwitchController

CONTROLLER = 'scaled_joint_trajectory_controller'
LEASE_S = .25


class Heartbeat:
    def __init__(self):
        self.armed = False
        self.last = None
        self.buffer = b''

    def feed(self, data, now):
        self.buffer += data
        if len(self.buffer) > 65536:
            raise ValueError('Heartbeat buffer exceeded')
        while b'\n' in self.buffer:
            line, self.buffer = self.buffer.split(b'\n', 1)
            kind, raw = line.decode().split()
            stamp = float(raw)
            if kind not in ('A','H') or not 0 <= now-stamp < 60:
                raise ValueError('Invalid heartbeat')
            if kind == 'A':
                self.armed = True
            if self.last is None or stamp > self.last:
                self.last = stamp

    def expired(self, now):
        return self.armed and (self.last is None or now-self.last > LEASE_S)


class Guard(Node):
    def __init__(self, child, fd, record):
        super().__init__('ur5e_independent_bench_guard')
        self.child, self.fd, self.record = child, fd, record
        self.heartbeat = Heartbeat()
        self.stopping = False
        self.done = False
        self.confirmed = False
        self.request = self.switch_request = None
        self.list_client = self.create_client(ListControllers,'/controller_manager/list_controllers')
        self.switch_client = self.create_client(SwitchController,'/controller_manager/switch_controller')
        self.create_timer(.02,self.tick)
        self.log('guard_started', child_pid=child.pid, lease_s=LEASE_S)

    def log(self,event,**values):
        self.record.write(json.dumps(dict(event=event,monotonic_s=time.monotonic(),**values))+'\n')
        self.record.flush()

    def stop(self,reason):
        if self.stopping:
            return
        self.stopping=True
        self.deadline=time.monotonic()+4
        self.log('guard_stop',reason=reason,armed=self.heartbeat.armed)
        if self.child.poll() is None:
            self.child.kill()
        if not self.heartbeat.armed:
            self.done=True
            self.confirmed=True  # No activation request was issued by this child.

    def tick(self):
        if self.done:
            return
        now=time.monotonic()
        if not self.stopping:
            try:
                while True:
                    data=os.read(self.fd,4096)
                    if not data:
                        break
                    self.heartbeat.feed(data,time.monotonic())
            except BlockingIOError:
                pass
            except (ValueError,OSError,UnicodeError):
                self.heartbeat.armed=True  # Cannot establish that activation never happened.
                self.stop('invalid heartbeat channel')
            if self.child.poll() is not None:
                self.stop('command process exited')
            elif self.heartbeat.expired(now):
                self.stop('command process heartbeat expired')
        if not self.stopping or self.done:
            return
        if now>self.deadline:
            self.log('STOP_UNCONFIRMED')
            self.get_logger().error('STOP UNCONFIRMED: use pendant Stop')
            self.done=True
            return
        if self.switch_request is not None:
            if self.switch_request.done():
                result=self.switch_request.result()
                if not result or not result.ok:
                    self.log('deactivation_rejected')
                    self.done=True
                else:
                    # Verify inventory after switch acknowledgement.
                    self.switch_request=None
                    self.request=None
            else:
                return
        if self.request is None:
            if self.list_client.service_is_ready():
                self.request=self.list_client.call_async(ListControllers.Request())
            return
        if not self.request.done():
            return
        result=self.request.result()
        state=next((c.state for c in result.controller if c.name==CONTROLLER),None) if result else None
        if state=='inactive':
            self.confirmed=True
            self.done=True
            self.log('inactive_verified',child_exit_code=self.child.poll())
        elif state=='active' and self.switch_client.service_is_ready():
            request=SwitchController.Request()
            request.deactivate_controllers=[CONTROLLER]
            request.strictness=SwitchController.Request.STRICT
            request.timeout.sec=2
            self.switch_request=self.switch_client.call_async(request)
            self.log('deactivation_requested')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',help='Required only for legacy fault/step tests')
    p.add_argument('--segmented',action='store_true',help='Guard smooth position tracking in a fixed workspace')
    p.add_argument('--step-mm',type=float,default=None,help='Deprecated: smooth control does not execute fixed-size steps')
    p.add_argument('--budget-mm',type=float,default=100.0,help='Fixed session workspace radius in mm (never auto-rebased)')
    p.add_argument('--scale',type=float,default=.75,help='Robot/hand displacement ratio')
    p.add_argument('--mode',choices=['position'],default='position',help='Direct clutched position tracking')
    args,extra=p.parse_known_args()
    if not args.segmented and not args.session:p.error('Fault tests require --session')
    if args.segmented and args.step_mm is not None:
        print('Ignoring legacy --step-mm: smooth tracking uses bounded rolling trajectories.',flush=True)
    if os.environ.get('ROS_DOMAIN_ID')!='43' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('Requires domain 43, localhost only')
    root=Path(__file__).resolve().parents[2]
    reader,writer=os.pipe();os.set_blocking(reader,False)
    env=dict(os.environ,UR5E_GUARD_FD=str(writer))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    output=root/'diagnostics/real-robot'/time.strftime('guard-%Y%m%d-%H%M%S.jsonl')
    with output.open('x') as record:
        command=([sys.executable,'-m','real_robot.continuous','--budget-mm',str(args.budget_mm),'--mode',args.mode,'--scale',str(args.scale)]+extra if args.segmented else
                 [sys.executable,'-m','real_robot.bench_fault_test','--fault','sigkill','--session',args.session]+extra)
        child=subprocess.Popen(command,env=env,pass_fds=(writer,))
        os.close(writer)
        node=Guard(child,reader,record)
        signal.signal(signal.SIGINT,lambda *_:node.stop('operator interrupted guard'))
        signal.signal(signal.SIGTERM,lambda *_:node.stop('guard termination requested'))
        try:
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node,timeout_sec=.02)
        finally:
            if child.poll() is None:
                child.kill()
            child.wait(timeout=3)
            os.close(reader)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        if not node.confirmed:
            raise RuntimeError('Independent stop not confirmed; use pendant Stop')
    print(f'Guard record: {output}')


if __name__=='__main__':main()
