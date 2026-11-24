"""Offline process-worker replay of a recording; no ROS nodes or robot sockets."""
import argparse
import json
import multiprocessing
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np
import xacro

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from real_robot import planning


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('record',type=Path)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    rows=[json.loads(line) for line in args.record.read_text().splitlines()]
    origin=next(row for row in rows if row['event']=='fixed_session_origin')
    goals=[row for row in rows if row['event']=='rolling_goal'][-30:]
    manifest=json.loads((ROOT/'config/real_robot_commissioning.json').read_text())
    xml=xacro.process_file('/opt/ros/humble/share/ur_description/urdf/ur.urdf.xacro',mappings={
        'ur_type':'ur5e','name':'ur','robot_ip':manifest['robot_ipv4'],
        'kinematics_params':str(ROOT/'calibration/ur5e_calibration.yaml')}).toxml()
    results=[]
    with ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn')) as pool:
        pool.submit(planning.initialize,xml,manifest['expected_calibration_hash'],
                    manifest['robot_ipv4'],np.array(origin['q'])).result(timeout=15)
        for goal in goals:
            request=dict(reference=tuple(np.array(goal[k]) for k in ('start','velocity','acceleration')),
                         target=np.array(goal['target']),origin=np.array(origin['pose']),workspace_m=.1)
            started=time.monotonic();future=pool.submit(planning.plan,request)
            callbacks=0
            while not future.done():
                # A stand-in for the main process servicing feedback. There is
                # no wait on the expensive planner in the main control loop.
                callbacks+=1
                time.sleep(.005)
            result=future.result()
            results.append(dict(sequence=goal['sequence'],worker_ms=result['planning_s']*1000,
                                elapsed_ms=(time.monotonic()-started)*1000,main_polls=callbacks))
    values=[row['elapsed_ms'] for row in results]
    report=dict(record=str(args.record),count=len(results),median_ms=float(np.median(values)),
                max_ms=max(values),over_100ms=sum(v>=100 for v in values),results=results)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='results'}))


if __name__=='__main__':main()
