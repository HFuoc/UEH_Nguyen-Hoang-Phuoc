"""Build a concise evidence-based PDF. Missing evidence stays visibly pending."""
import json
from pathlib import Path
import fitz

ROOT=Path(__file__).resolve().parents[1]
GENERATED=ROOT/'analysis/generated'
doc=fitz.open()


def page(title, paragraphs):
    p=doc.new_page(width=595,height=842)
    p.draw_rect(fitz.Rect(0,0,595,9),color=None,fill=(.10,.25,.35))
    p.insert_text((44,55),title,fontsize=20,color=(.10,.25,.35))
    y=85
    for paragraph in paragraphs:
        rect=fitz.Rect(44,y,550,780)
        # Use a temporary shape to calculate the actual occupied height.
        shape=p.new_shape()
        spare=shape.insert_textbox(rect,paragraph,fontsize=11,lineheight=1.45)
        if spare<0: raise RuntimeError('Report text overflow: '+title)
        shape.commit()
        used=rect.height-spare
        y+=used+16
    p.insert_text((44,815),f'CRC 2026 | Sensor-only baseline | {len(doc)}',fontsize=9,color=(.4,.4,.4))
    return p,y


runs=json.loads((GENERATED/'all_runs.json').read_text()) if (GENERATED/'all_runs.json').exists() else []
page('CRC 2026 - Technical report',[
    'DRAFT - Personal review and final evaluation are pending.\nName: Nguyen Hoang Phuoc\nUniversity: University of Economics Ho Chi Minh City (UEH)\nSchool: School of Technology and Design\nInstitute: Institute of Intelligent and Interactive Technologies\nStudent ID: 31231021201',
    'Goal: drive using onboard camera, LiDAR and wheel odometry. This submission prioritizes a small, explainable controller and conservative stopping. It does not claim complete track coverage or a competition score.',
    'Evidence policy: results in this document come only from recorded driver telemetry and images. Synthetic unit tests establish isolated behaviour, not successful driving. Missing evaluation results are stated explicitly.',
    'Submission status: the participant must review this report, complete the identity and AI disclosure, and record the required continuous English video.'])
p,y=page('System architecture',[
    'The Python package crc_solution is independent of the official simulator package. One node owns motion commands. No simulator entity service, ground-truth stream or route-coordinate file is used.',
    'Image and calibration -> lane and sign observations\nLiDAR -> swept body corridor clearance\nOdometry -> heading, stationary speed and travelled distance\nObservations -> safety and traffic behaviour -> curvature control -> /cmd_vel',
    'A wall-clock timer executes the control loop so loss of the simulation clock still produces zero velocity. Behaviour durations use simulation time. Images are processed only when a new frame arrives; repeated control ticks do not count as fresh detections.',
    'The launch interface is ros2 launch crc_solution run.launch.py. The simulator is already running, and dependencies and workspace setup are completed before this command.'])
if y < 610:
    boxes=[(fitz.Rect(44,y+10,185,y+62),'Camera + calibration'),
           (fitz.Rect(44,y+82,185,y+134),'LiDAR + odometry'),
           (fitz.Rect(220,y+10,375,y+62),'Lane / sign perception'),
           (fitz.Rect(220,y+82,375,y+134),'Safety + behaviour'),
           (fitz.Rect(410,y+82,550,y+134),'/cmd_vel')]
    for rect,label in boxes:
        p.draw_rect(rect,color=(.1,.25,.35),fill=(.94,.97,.98))
        p.insert_textbox(rect+fitz.Rect(5,16,-5,-5),label,fontsize=10,align=1)
    for start,end in [((185,y+36),(220,y+36)),((298,y+62),(298,y+82)),
                      ((185,y+108),(220,y+108)),((375,y+108),(410,y+108))]:
        p.draw_line(start,end,color=(.1,.25,.35),width=1.2)
        x,z=end
        if start[1]==end[1]:
            p.draw_line((x-5,z-3),end,color=(.1,.25,.35));p.draw_line((x-5,z+3),end,color=(.1,.25,.35))
        else:
            p.draw_line((x-3,z-5),end,color=(.1,.25,.35));p.draw_line((x+3,z-5),end,color=(.1,.25,.35))
page('Lane perception and steering',[
    'White lane markings are selected using brightness, low saturation and local contrast. Multiple horizontal image bands provide candidates. Pairs are checked against projected lane width; a single visible boundary gives lower confidence. A small consensus fit rejects isolated candidates from crossing marks. A broad bright ramp surface can supply its visible right edge when paint is occluded.',
    'The stock camera intrinsics and mounting height project pixels onto a locally flat ground plane. A line fit estimates lateral offset and heading. Consistency across bands determines confidence. The inferred target is in the robot frame, not a world coordinate.',
    'The angular command is linear speed multiplied by a curvature estimate toward the look-ahead target. Positive target offset is left of the robot, giving positive angular velocity. Speed falls with curvature and lower confidence.',
    'Brief missing markings reuse the most recent lane estimate with an odometry yaw correction. This prediction is bounded in time. A longer perception loss stops motion. Ground projection is approximate on a ramp; this is a known failure mode.'])
page('Traffic and obstacle behaviour',[
    'STOP candidates use red colour and a compact polygonal shape. Warning triangles are rejected. Several distinct frames are required. Approximate apparent sign size triggers the stop approach. The hold timer starts when measured wheel-odometry speed is almost zero and runs for at least two simulation seconds.',
    'Traffic lights require a coloured compact region and a dark vertical housing. Temporal confirmation reduces single-frame errors. Red and yellow cause stopping before a junction. A short odometry-based crossing interval permits clearing a junction already entered on green.',
    'LiDAR rays are transformed using the scan angle origin and increment. Returns ahead of the nose in the projected body corridor cause stopping. Returns inside the stock body footprint are excluded because pitching on the ramp can expose the wheels to the scanner. Invalid front-sector data is not interpreted as free space. Clearance must persist before motion resumes.',
    'Safety stopping has priority over forward progress. The driver does not recognize every sign type and does not overtake. These limitations must remain visible in the participant explanation.'])
page('Environment and reproducibility',[
    'The supplied environment uses ROS 2 Humble and Gazebo Classic 11 in Docker. The host in this session is Windows with an existing Ubuntu 20.04 VMware guest, configured for 8 GB RAM and 8 virtual CPUs.',
    'The official simulator world, model and configuration files are protected by SHA-256 hashes. tools/check_rules.py verifies them and scans the submitted controller for forbidden interfaces. The Dockerfile uses HTTPS repositories to accommodate this network.',
    'Build and install instructions are in README.md. Runtime dependencies are ROS Python bindings, cv_bridge, NumPy and OpenCV. Analysis uses matplotlib and PyMuPDF on the host.',
    'Clean-environment validation must be distinguished from a fresh build in a reused container. A clean-machine claim is only justified after its corresponding check has actually run. See PROGRESS.md for the current status.'])
validation=json.loads((GENERATED/'validation.json').read_text()) if (GENERATED/'validation.json').exists() else None
validation_note=('Local automated checks recorded at '+validation['utc']+'. '+
    '; '.join(c['command']+': '+('PASS' if c['exit_code']==0 else 'FAIL') for c in validation['checks'])) if validation else 'Local automated check record not yet generated.'
page('Validation method',[
    validation_note,
    'Unit tests cover projected lane geometry, dim lanes, blank imagery, STOP versus warning triangles, lamp housing, missing sensors, invalid LiDAR, scan angle conventions, bounded lane prediction, obstacle clearance dwell and traffic behaviour.',
    'Simulation evaluation targets three independent runs at track_scale=1.0 with all props, signs and lights enabled, plus two valid changed start poses. Each evaluation is bounded to five simulation minutes and stopped early after 45 simulation seconds without progress. Different runs may have different traffic phases.',
    'CSV telemetry contains time, state, commanded and measured speed, odometry, lane confidence, estimated lane offset and detected traffic controls. Annotated frames are saved periodically and at state transitions.',
    'analysis/summarize.py produces all metric plots from the original CSVs. Estimated lane RMS is not official ground-truth lane RMS; integrated wheel distance is not the route-completion score. No collision or traffic compliance count is inferred without a verified annotation method.'])
if runs:
    paragraphs=['Recorded runs below are measurements, not competition scores.']
    for r in runs[:8]:
        rms='unavailable' if r['lane_estimate_rms_m'] is None else f"{r['lane_estimate_rms_m']:.4f} m"
        paragraphs.append(f"{Path(r['source']).parent.name}: duration {r['duration_sim_s']:.1f} simulation s; wheel distance {r['distance_wheel_odom_m']:.3f} m; camera-estimated lane RMS {rms}. States: {', '.join(r['states_seen'])}.")
    page('Recorded results',paragraphs)
else:
    page('Recorded results - pending',[
        'No real-run telemetry was available when this PDF was generated. No distance, lane RMS, success rate, collision count or competition score is claimed.',
        'Run the evaluation, export /tmp/crc_results from the container, execute analysis/summarize.py results, then regenerate this PDF with analysis/build_report.py. Do not replace missing observations with expected values.'])
plots=[GENERATED/Path(r['source']).parent.name/'run.png' for r in runs]
plots=[p for p in plots if p.exists()]
if plots:
    p,y=page('Measured run plots',['Representative plot. Source and metric definitions are included in the generated summary next to the figure.'])
    p.insert_image(fitz.Rect(40,y,555,min(y+420,770)),filename=str(plots[-1]))
else:
    page('Evidence figures - pending',[
        'The plot page is intentionally empty until measured telemetry exists. The analysis script creates an odometry trajectory, linear-speed plot, camera lane-offset plot and perception/steering plot.',
        'Images in the simulator documentation are reference material, not results of this solution. Only frames captured during this solution running may be described as experimental evidence.'])
images=[]
if runs:
    best=max(runs,key=lambda r:r['distance_wheel_odom_m'])
    images=sorted((ROOT/Path(best['source']).parent).glob('*_FOLLOW.jpg'))
if images:
    p,y=page('Recorded camera evidence',[
        'Actual onboard camera frame saved by the driver. Yellow dots show inferred lane centres; coloured boxes show detections. Overlay labels are controller estimates, not independently verified annotations.',
        str(images[-1].relative_to(ROOT))])
    p.insert_image(fitz.Rect(44,y,550,min(y+385,780)),filename=str(images[-1]))
page('Limitations and next experiments',[
    'Tight lane clearance amplifies calibration and boundary-selection errors. Intersections, crosswalk stripes and multiple parallel markings can create ambiguous candidates. The baseline selects visible continuity; it has no full route planner.',
    'Camera projection assumes approximately level ground. Ramp pitch and tunnel flicker can reduce confidence. A stop is safer than a blind recovery but reduces distance covered.',
    'Small or oblique STOP plates and lamps can be missed or misclassified. Temporal confirmation cannot recover an object that is never detected. Red-light stopping can persist if the lamp is lost from view.',
    'Stationary obstacles stop progress because overtaking is disabled. Bus-stop, highway and warning-sign-specific actions are incomplete. There is no validated automatic FINISH detector.',
    'Next experiments should focus on real failure frames, camera-ground calibration and short controlled approaches to each traffic object before attempting a more complex lane-change controller.'])
page('AI assistance and personal review',[
    'AI-generated work in this session: the initial package implementation, unit tests, analysis utilities, environment automation, README, report generator and video rehearsal outline. The AI also inspected supplied simulation assets during development.',
    'Participant-authored or modified work: PENDING participant declaration. Record exactly what you changed, why you changed it, and what you personally tested. Do not claim sole authorship of generated code.',
    'Participant understanding checklist: explain the sign of angular velocity; show how lane pixels become a target; explain sensor timeout and STOP hold; change max_speed live; distinguish an estimated metric from official scoring; describe at least one observed failure.',
    'References: official UEH CRC 2026 Simulation Guideline and Simulation Pack supplied by the organizer; ROS 2 Humble interfaces used by the starter; Docker Ubuntu installation documentation, https://docs.docker.com/engine/install/ubuntu/.',
    'This draft must be personally reviewed before submission. The participant must supply the unedited six-minute video and confirm the repository identity and submission deadline.'])
doc.save(ROOT/'REPORT.pdf')
print(f'Wrote REPORT.pdf ({len(doc)} pages). Real-run summaries: {len(runs)}')
