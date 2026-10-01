"""Summarize real driver CSVs. These are sensor estimates, not official scores."""
import argparse
import csv
import json
import statistics
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def summarize(csv_path, output):
    with csv_path.open() as stream:
        rows=list(csv.DictReader(stream))
    rows=[row for row in rows if float(row['sim_time'])>0]
    if not rows:
        return None
    def series(name):
        return np.array([float(row[name]) for row in rows])
    t=series('sim_time'); t=t-t[0]
    confidence=series('lane_confidence')
    valid=confidence >= .35
    lateral=series('lane_lateral_est_m')
    provenance=csv_path.parent/'provenance.json'
    case=json.loads(provenance.read_text())['case'] if provenance.exists() else next(
        (p.name for p in csv_path.parents if p.name.startswith(('default_','shifted_'))),'development')
    metrics={'source':csv_path.as_posix(),'case':case,'samples':len(rows),'duration_sim_s':float(t[-1]),
             'distance_wheel_odom_m':float(series('distance_odom_m')[-1]),
             'lane_estimate_rms_m':float(np.sqrt(np.mean(lateral[valid]**2))) if valid.any() else None,
             'lane_confident_sample_fraction':float(np.mean(valid)),
             'states_seen':sorted(set(row['state'] for row in rows)),
             'official_score':None,
             'note':'Wheel odometry path length is not route completion. Camera lane RMS is not ground-truth RMS.'}
    output.mkdir(parents=True,exist_ok=True)
    (output/'summary.json').write_text(json.dumps(metrics,indent=2)+'\n')
    fig,axes=plt.subplots(2,2,figsize=(10,7),constrained_layout=True)
    axes[0,0].plot(series('x_odom'),series('y_odom'))
    axes[0,0].set(title='Wheel odometry trajectory',xlabel='x (m)',ylabel='y (m)',aspect='equal')
    axes[0,1].plot(t,series('command_v'),label='command')
    axes[0,1].plot(t,series('speed_measured'),label='odometry',alpha=.7)
    axes[0,1].set(title='Linear speed',xlabel='Simulation time (s)',ylabel='m/s'); axes[0,1].legend()
    axes[1,0].plot(t,np.where(valid,lateral,np.nan))
    axes[1,0].set(title='Camera lane offset estimate',xlabel='Simulation time (s)',ylabel='m')
    axes[1,1].plot(t,confidence,label='lane confidence')
    axes[1,1].plot(t,series('command_w'),label='angular command (rad/s)',alpha=.7)
    axes[1,1].set(xlabel='Simulation time (s)',title='Perception and steering'); axes[1,1].legend()
    for ax in axes.flat: ax.grid(alpha=.2)
    fig.savefig(output/'run.png',dpi=140); plt.close(fig)
    return metrics


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('results',type=Path)
    parser.add_argument('--out',type=Path,default=Path('analysis/generated'))
    args=parser.parse_args()
    reports=[]
    for path in sorted(args.results.rglob('telemetry.csv')):
        result=summarize(path,args.out/path.parent.name)
        if result: reports.append(result)
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'all_runs.json').write_text(json.dumps(reports,indent=2)+'\n')
    defaults=[r['distance_wheel_odom_m'] for r in reports if r['case'].startswith('default_')]
    (args.out/'batch_summary.json').write_text(json.dumps({
        'cases':len(reports),'default_cases':len(defaults),
        'median_default_wheel_distance_m':statistics.median(defaults) if defaults else None,
        'note':'This median is measured wheel path length, not the official performance score.'},indent=2)+'\n')
    print(json.dumps(reports,indent=2))


if __name__=='__main__': main()
