# Participant video — pending recording

Video URL: PENDING

Participant: Nguyen Hoang Phuoc, University of Economics Ho Chi Minh City (UEH), School of Technology and Design, Institute of Intelligent and Interactive Technologies.

Record exactly one continuous six-minute take. Speak English, show your face in a corner, use normal playback speed and do not edit or splice takes.

| Time | Demonstration |
| --- | --- |
| 0:00–0:30 | State full name, school and faculty. |
| 0:30–2:00 | Explain image lane boundaries, steering direction, behaviour priority and sensor watchdog in your own words while showing code. |
| 2:00–4:00 | Show the simulator and driver live. Explain what actually happens, including a failure if one occurs. |
| 4:00–5:00 | Change `max_speed` from 0.12 to 0.08 with `ros2 param set /crc_driver max_speed 0.08`; demonstrate/re-run and explain the observed difference. |
| 5:00–6:00 | Describe limitations: perception errors, tight lanes, ground projection and disabled overtaking. |

Understand before recording: positive angular velocity turns left; STOP timing begins only when odometry reports almost zero speed; missing sensors trigger zero velocity; no ground-truth topic is used. Explain why a camera confidence estimate is not an official scoring metric.

Use this outline for rehearsal, not a script to read. Recording and personal explanation must be performed by the participant.
