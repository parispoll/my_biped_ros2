# Checks

Inspect model/sensor behavior and report diagnostics.

Run tools from the workspace root with `python3 -m scripts.checks.<tool> --help`. Direct file execution also works.

| Tool | Purpose |
|---|---|
| [check_contact_sampler_shutdown.py](check_contact_sampler_shutdown.py) | Stress the real ROS sampler under traffic and alternating SIGINT/SIGTERM. |
| [check_contacts.py](check_contacts.py) | Check fresh ground-contact evidence from both feet in a settled crouch. |
| [check_imu.py](check_imu.py) | Verify an ideal, stationary pelvis IMU on /biped/imu; use matching ROS_DOMAIN_ID. |
| [check_roll_model.py](check_roll_model.py) | Expand/check the ten-joint prototype and draw a front-view geometry preview. |
| [check_standing.py](check_standing.py) | Bounded pose check; use the same GZ_PARTITION as the simulator. |
| [diagnose_ankle.py](diagnose_ankle.py) | Compare pre-push and final-second ankle terms with a static gravity estimate. |
