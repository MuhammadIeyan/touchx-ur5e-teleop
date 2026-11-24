#!/usr/bin/env python3
"""Summarize observation coverage and jitter; never label preview error as accuracy."""
import argparse,json,gzip
from collections import Counter
from pathlib import Path
import numpy as np

def summarize(rows):
    if not rows:raise ValueError('Empty recording')
    tcp=np.asarray([r['tcp'][:3] for r in rows if r.get('tcp') is not None])
    reasons=Counter(r['reason'] for r in rows)
    result={'mode':'preview_only','samples':len(rows),
            'duration_s':rows[-1]['monotonic_s']-rows[0]['monotonic_s'],
            'state_counts':dict(reasons),'valid_tcp_samples':len(tcp),
            'preview_samples':sum(r.get('preview_xyz') is not None for r in rows),
            'accuracy_assessed':False,
            'note':'Preview targets were not sent to the robot. TCP variation is not absolute positioning accuracy.'}
    if len(tcp):
        result['tcp_span_mm']=(np.ptp(tcp,axis=0)*1000).tolist()
        result['tcp_std_mm']=(np.std(tcp,axis=0)*1000).tolist()
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('recording',type=Path);a=p.parse_args()
    text=gzip.open(a.recording,'rt').read() if a.recording.suffix=='.gz' else a.recording.read_text()
    rows=[json.loads(line) for line in text.splitlines() if line.strip()]
    if any(r.get('mode')!='preview_only' for r in rows):p.error('Expected preview_only recording')
    print(json.dumps(summarize(rows),indent=2,allow_nan=False))
if __name__=='__main__':main()
