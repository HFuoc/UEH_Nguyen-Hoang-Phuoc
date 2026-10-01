"""Generate report figures directly from the selected recorded trial."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def generate(case='video_v15'):
    evidence = ROOT/'analysis/evidence'/case
    source = next(evidence.rglob('telemetry.csv'))
    rows = list(csv.DictReader(source.open()))
    out = ROOT/'analysis/figures'
    out.mkdir(exist_ok=True)
    values = lambda key: np.array([float(r[key]) for r in rows])
    t = values('sim_time'); t -= t[0]
    dist = values('distance_odom_m')
    dt = np.maximum(0., np.diff(t, append=t[-1]))
    states = {s: float(sum(dt[i] for i,r in enumerate(rows) if r['state']==s))
              for s in sorted({r['state'] for r in rows})}
    metrics = {'case':case, 'source':source.relative_to(ROOT).as_posix(),
               'distance_odom_m':float(dist[-1]), 'logged_duration_s':float(t[-1]),
               'final_state':rows[-1]['state'], 'state_duration_s':states,
               'processing_median_ms':float(np.median(values('processing_ms'))),
               'processing_p95_ms':float(np.percentile(values('processing_ms'),95)),
               'estimated_lateral_rms_m':float(np.sqrt(np.mean(values('lane_lateral_est_m')[values('lane_confidence')>=.28]**2)))}
    (out/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    plt.rcParams.update({'font.family':'Times New Roman','font.size':10})
    fig,ax=plt.subplots(2,1,figsize=(7,4.2),sharex=True)
    ax[0].plot(t,dist,color='#1b4965'); ax[0].set_ylabel('Wheel-odometry path (m)')
    ax[1].plot(t,values('command_v'),label='Command',lw=1)
    ax[1].plot(t,values('speed_measured'),label='Measured',lw=.8,alpha=.7)
    ax[1].set_ylabel('Forward speed (m/s)'); ax[1].set_xlabel('Simulation elapsed time (s)'); ax[1].legend()
    for a in ax: a.grid(alpha=.2)
    fig.tight_layout(); fig.savefig(out/'distance_speed.png',dpi=180); plt.close(fig)
    fig,ax=plt.subplots(2,1,figsize=(7,4.2),sharex=True)
    ax[0].plot(t,values('lane_confidence')); ax[0].axhline(.28,color='r',ls='--',lw=.7)
    ax[0].set_ylabel('Lane confidence')
    ax[1].plot(t,values('lane_lateral_est_m')); ax[1].set_ylabel('Estimated lateral offset (m)')
    ax[1].set_xlabel('Simulation elapsed time (s)')
    for a in ax: a.grid(alpha=.2)
    fig.tight_layout(); fig.savefig(out/'lane_estimates.png',dpi=180); plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,3.2)); ax.barh(list(states),list(states.values()),color='#1b4965')
    ax.set_xlabel('Recorded state duration (simulation s)'); fig.tight_layout()
    fig.savefig(out/'states.png',dpi=180); plt.close(fig)
    fig,ax=plt.subplots(figsize=(6,3.4)); ax.plot(values('x_odom'),values('y_odom'))
    ax.scatter([values('x_odom')[0],values('x_odom')[-1]],[values('y_odom')[0],values('y_odom')[-1]],c=['green','red'])
    ax.set(xlabel='Odometry x (m)',ylabel='Odometry y (m)',title='Wheel-odometry trajectory; no ground-truth alignment')
    ax.axis('equal'); ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(out/'trajectory.png',dpi=180); plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,2.8)); ax.set_xlim(0,10); ax.set_ylim(0,4); ax.axis('off')
    boxes=[(.1,2.6,'Camera + calibration'),(.1,.7,'LiDAR + odometry'),(3.7,2.6,'Lane, signs, lights'),(3.7,.7,'Freshness + clearance'),(7.1,1.65,'Behaviour arbitration\nand /cmd_vel')]
    for x,y,label in boxes: ax.text(x+1.35,y+.35,label,ha='center',va='center',bbox=dict(boxstyle='round,pad=.65',fc='#eef3f7',ec='#1b4965'))
    for a,b in [((2.8,2.95),(3.4,2.95)),((2.8,1.05),(3.4,1.05)),((6.45,2.95),(7.1,2.1)),((6.45,1.05),(7.1,1.9))]: ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->'))
    fig.tight_layout(); fig.savefig(out/'architecture.png',dpi=180); plt.close(fig)
    import cv2
    fig,axes=plt.subplots(2,2,figsize=(7,5))
    for ax,(name,label) in zip(axes.ravel(),[
        ('tunnel_bend.jpg','Tunnel bend'),('curve_boundary.jpg','Ambiguous curved corridor'),
        ('sign_stop_real.jpg','Recorded STOP plate'),('sign_crosswalk_real.jpg','Recorded crossing sign')]):
        frame=cv2.imread(str(ROOT/'tests/data'/name))
        ax.imshow(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)); ax.set_title(label); ax.axis('off')
    fig.tight_layout(); fig.savefig(out/'recorded_images.png',dpi=180); plt.close(fig)
    return metrics

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--case',default='video_v15')
    print(json.dumps(generate(parser.parse_args().case)))
