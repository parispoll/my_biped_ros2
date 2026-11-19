# Longer and faster finite walking

This extends the verified three-placement, 50 mm baseline. A profile independently chooses placement count, nominal distance ahead of the support foot, and movement duration. These are finite quasi-static trajectories; continuous dynamic walking and disturbance recovery during walking remain future work.

## What the settings mean

`WalkingProfile` in `motion/walking_profile.py` accepts 3–20 placements, 50/75/100 mm nominal step length, and timing scales 1, 0.5 or 0.25. A 75 mm first placement moves the swing foot 75 mm from the initial abreast stance. Later placements move the trailing foot 150 mm, landing 75 mm ahead of the support foot.

Ten 75 mm placements predict left/right final advances of 750/675 mm and approximately 712.5 mm forward COM progress. Both legs alternate; this counts foot placements, rather than left/right pairs.

`motion/profiled_walk.py` maps the original geometric plan onto a shorter clock. Scale 0.25 makes movement phases four times faster. Baseline, unloading confirmation, recontact and final holds retain minimum durations so independent measurements can still validate them. The resulting ten-placement trajectory lasts 585 simulated seconds after readiness. This is considerably faster than our educational baseline, but still slow walking.

Motor torque caps, physical motor speed limits, contact freshness, sole geometry, support margin, joint tracking and IMU audit tolerances remain unchanged. Faster profiles explicitly use a 0.25 rad/s measured joint-speed envelope, replacing the baseline's conservative 0.05 rad/s experiment envelope. The physical motor limit remains 2.4 rad/s. Predicted peak reference speed for the 75 mm, scale 0.25 plan is 0.1883 rad/s.

## Code responsibilities

- `motion/alternating_walk.py`: support exchanges, retained world foot positions, repeated gait geometry. Identical repeated cycles reuse a checked pose template translated along world X; flat ground geometry, joint angles and static torques are invariant under that translation.
- `motion/first_step.py`: initial placement, with parameterized distance.
- `motion/walking_profile.py`: supported experiment settings and explicit speed envelope.
- `motion/profiled_walk.py`: phase clock mapping, protected measurement holds, guard routing.
- `validation/alternating_walk_audit.py`: independent validation of every placement and final stop.
- `scripts/experiments/run_lateral_shift.py`: launch, recording, shutdown, source hash check and report.
- `tools/force_panel/`: exact-profile preflight requirement, GUI fields, launch command and progress.

## Reproduce through the panel

Open http://127.0.0.1:8765/. Choose alternating-walk OFFLINE preflight, then select the count, distance and timing. Generate that exact preflight. After it passes, select alternating walking steps with the same settings and run. Each combination needs its own preflight; changing count, distance or timing invalidates the match. Every run gets a new evidence folder.

## Validation record

The 100 mm candidate was rejected offline by the current first-step reference-speed check. This is a planning-envelope rejection, not a measured motor or balance limit. The ten-placement 75 mm / 0.25 candidate passed offline geometry, gravity, timing and synthetic guard tests. Live measured results are recorded separately below when available.

GUI preflight `20261114-104209-c8ea41b8` passed for three 75 mm placements at scale 0.25. Its live trial `20261114-104343-d3a56829` stopped during the first unloading transfer at elapsed 26.948 s, before any swing. A later contact sample summed to 232.67 N, outside the original ±5% band around 218.57 N. This candidate is rejected; offline checks do not establish physical success. A scale 0.5 candidate is being evaluated separately.

Three-step scale 0.5 preflight `20261114-104738-d54411e8` passed. Measured run `20261114-104936-07c14374` then passed all audits: three placements, 187.573 mm forward COM progress, minimum horizontal-swing clearance 29.980 mm, peak measured joint speed 0.07561 rad/s, maximum joint tracking error 0.001576 rad, peak motor utilization 53.43%, no saturated samples, source unchanged. It completed in 339 simulated seconds after readiness, compared with 669 s for the old three-step 50 mm baseline. On this equal-count comparison, average forward COM progression is approximately 2.96× faster, combining larger steps with shorter movement durations.

Ten-step scale 0.5 preflight `20261114-105830-3fd895ac` passed. Live recording: `20261114-105944-c4c3904f` — **passed all six audits**.


## Verified ten-step result — 14 November 2026

GUI run `20261114-105944-c4c3904f` completed all ten placements and a stable final double-support stop. The source snapshot matched at completion; Gazebo, controller and observers closed normally.

| Measurement | Result |
|---|---:|
| Verified placements | 10 / 10 |
| Nominal landing ahead of support | 75 mm (previously 50 mm) |
| Measured first foot travel | 75.022 mm |
| Measured subsequent foot travel | 150.050–150.283 mm |
| Forward COM progress | 712.776 mm |
| Duration after readiness | 1,095 simulated s |
| Minimum horizontal-swing sole clearance | 29.980 mm |
| Minimum supporting-sole COM margin | 59.175 mm |
| Maximum sole-height tracking error | 0.352 mm |
| Maximum foot XY tracking error | 0.479 mm |
| Peak measured joint speed | 0.07548 rad/s |
| Maximum joint tracking error | 0.001582 rad |
| Peak torque-cap utilization | 53.43% |
| Saturated controller samples | 0 |
| Largest independent COM sample gap | 21 ms |

Every placement passed fresh no-contact evidence while raised, landing contact, clearance, support margin and final load-share checks. Overall controller, contacts/geometry, sensors, COM coverage, physical damping and alternating-placement audits all passed.

The movement phases are 2× faster, and nominal steps are 50% larger. Average COM speed over this complete finite run is approximately 0.651 mm/s. This remains deliberately slow quasi-static walking. The matched three-placement comparison measured approximately 2.96× the old average forward progression rate; do not describe it as human-speed walking.

There is small measured placement error (up to 0.283 mm in foot travel) and the maximum measured XY error is 0.479 mm, inside the retained 1 mm tolerance. We have not established indefinite drift-free walking, uneven-ground walking, turning or push recovery while walking. The next development should address adaptive weight transfer and contact-aware continuous phase transitions before attempting another uniform timing reduction.

## Command-line reproduction

Run from the workspace after sourcing ROS Jazzy and `install/setup.bash`:

```bash
python3 -m scripts.planning.preview_alternating_walk \
  --walk-steps 10 --walk-step-mm 75 --walk-time-scale 0.5 \
  --output evidence/my-ten-step-preview

python3 -m scripts.experiments.run_lateral_shift --actuator-profile robotis_p54 --alternating-walk \
  --walk-steps 10 --walk-step-mm 75 --walk-time-scale 0.5 \
  --direction left --real-time-factor 1.0 --gui \
  --output evidence/my-ten-step-run
```

Use a new output folder on every run. The GUI's ten-step preset is the preferred route because it requires the matching preflight. Raw evidence and the generated `response.png` and `walking-replay.gif` are in the live recording folder; the GIF reconstructs measured geometry with compressed time and is not a Gazebo screen capture.
