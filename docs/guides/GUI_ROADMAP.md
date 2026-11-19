> **Latest verified milestone — 13 November 2026:** three alternating simulated steps (right → left → right) passed all six audits, with 30 mm clearance, 125.009 mm COM forward progress, stable double-support stop and no motor saturation. The independent COM recorder now buffers physics observations separately from marker drawing. See [the walking guide](ALTERNATING_WALK.md). This is a finite slow gait; longer/faster walking and disturbance recovery remain future work.

**Latest milestone (14 November 2026):** GUI run `20261114-105944-c4c3904f` verified ten 75 mm alternating placements at 2× movement speed, with 712.8 mm measured COM progression. Choose the verified ten-step preset. See [settings, measurements and reproduction](LONGER_FASTER_WALK.md). Earlier dated entries below are historical progression.

> **Latest measured milestone — 13 November 2026:** one 50 mm forward step with 30 mm clearance passed through the GUI (left support/right swing), including fresh airborne contact evidence, forward landing and 50/50 reload in a new staggered stance. Measured advance 50.004 mm, clearance 29.994 mm, peak motor utilization 53.1%, no saturation. See [the first-step guide](FIRST_FORWARD_STEP.md). Continuous alternating walking is next; mirrored live stepping remains untested.

# Latest measured milestone: calculated 30 mm step-clearance lift

The right foot lifted at least 30.024847 mm with left support, fresh no-contact
evidence, landing and full return. GUI job `20261111-181819-9790fdb3` passed.
The height was selected for an offline checked 50 mm forward-swing design;
forward placement is not yet executed. Right-support 30 mm remains unrun.
See [the clearance budget, code changes and measured evidence](STEP_CLEARANCE_30MM.md).
Earlier entries below retain their development history.

# Latest measured milestone: 2 mm foot lifts verified on both sides

Both 257-second lift/contact-loss/landing/return trials passed using identical
package sources. Minimum held clearances: 2.002620 mm (right foot lifted) and
2.002252 mm (left foot lifted). Neither run saturated a motor.
See [measured results, implementation and next boundary](LIVE_FOOT_LIFT.md).
Next: check larger clearance before adding forward placement. Earlier sections
below describe the development history and their former next-step statements.

# Latest measured milestone: first 2 mm foot lift, left support

GUI job `20261111-160340-2d4a7aff` passed lift, fresh no-contact evidence,
landing, reload and full return. Minimum held right-sole clearance was
2.002620 mm; supporting-sole COM margin was 59.992074 mm, with no saturation.
Right-support live lift is prepared but unrun. Next is that mirrored check.
See [the implementation, guard equations and measured result](LIVE_FOOT_LIFT.md).
The entries below retain earlier development stages.

# New preparation stage: 2 mm foot-lift OFFLINE preview

The GUI can now generate a checked kinematic/static lift preview for either
supporting side. It starts from the verified zero-load endpoint and does not
launch Gazebo or command motors. Offline checks are separate from measured
verification; live lift, landing and clearance audits remain unimplemented.
See [the equations, code map and next requirements](FIRST_FOOT_LIFT_PREVIEW.md).

# Current verified milestone: zero-load transitions on both sides

Left support and right support both passed the full zero-load hold and
return on 5 November 2026. The unloaded foot carried about 0.0035 N in each
trial, with approximately 60 mm COM margin inside the supporting sole.
Both trials retained light ground contact; no foot lift was commanded.
Next: prepare a small deliberate lift with measured clearance and support
checks. See [measured results and control explanations](ZERO_LOAD_TRANSITION.md).

# Current verified milestone: left-support zero-load transition

The left-support zero-load trial passed on 5 November 2026: right-foot held
force 0.003478 N, support COM margin 59.981 mm, full return, no saturation.
The physics-state contact bench also passed. Right-support zero load is
prepared but unrun; a deliberate foot lift remains a later step.
See [calculations, control changes and measured results](ZERO_LOAD_TRANSITION.md).
Use the GUI roadmap to select each mode and inspect its recorded evidence.

The entries below retain earlier development history; the current status
above supersedes their former next-step statements.

# Latest verified milestone: mirrored 20%, 10%, 5% unloading

Both supporting sides passed all three measured targets. See [results and next development boundary](STAGED_LOWERED_UNLOADING.md). Contact-aware zero load and a foot lift remain future work.

# Latest measured milestone: 20%, 10%, 5% right-foot load with left support

All three stages passed on 5 November 2026. See [measured results, calculations and implementation map](STAGED_LOWERED_UNLOADING.md). Mirrored right-support stages remain unrun; zero contact, foot lift and walking are not demonstrated.

# Active development: staged lowered unloading

Separate 20%, 10% and 5% remaining-load targets, with measured prerequisites. See [stage guide](STAGED_LOWERED_UNLOADING.md). Prior distance-based milestones retain their original guards.

# Latest verified milestone: 40 mm lowered bilateral transfer

Both 40 mm directions passed measured audits on 5 November 2026. See [results and code changes](LOWERED_TRANSFER_40MM.md). The prepared 30 mm stages remain unrun.

# Current task: 40 mm lowered bilateral transfer

User requested preparing and running both directions. See [40 mm guide](LOWERED_TRANSFER_40MM.md). The 30 mm stage remains unrun; only matching measured results verify a roadmap stage.

# Current next step: 30 mm lowered transfer

Both 20 mm directions passed measured audits. The 30 mm left/right stages are prepared, awaiting your runs. See [instructions and calculations](../history/NEXT_LOWERED_TRANSFER_30MM.md).

# One GUI for the development roadmap

Current user-run tests: [20 mm lowered lateral transfers](../history/NEXT_LOWERED_TRANSFER_20MM.md).
The new entries are prepared and await measured verification; experiments
are executed by the user.

The new **40 mm lowered lateral transfer** mode combines the crouch with a
5 mm left/right translation. See [LOWERED_LATERAL_TRANSFER.md](../history/LOWERED_LATERAL_TRANSFER.md)
for its schedule, equations, files, measured results and per-direction status.

Current envelope: **10, 20, 30 and 40 mm** centered lowering. The original
10/20 mm implementation and its measured results below remain historical
evidence; the 30 mm addition is described at the end of this guide.

Open **http://127.0.0.1:8765/**. If needed, restart it from the workspace with:

```bash
bash scripts/start_force_panel_background.sh
```

## How to use it

The roadmap lists measured milestones and future work. **Verified** is based
on an actual saved passing result, not on merely adding a button or an offline
calculation. **Choose** selects a runnable experiment; it does not start a run.
Review its controls and click **Run**. Planned stages have no launch button.

1. **Bilateral load transfer + small pulses** uses the existing P54 transfer
   mode. The loaded foot and lateral force direction remain independent.
2. **10 mm centered lowering + return** repeats the measured baseline.
3. **20 mm centered lowering + return** extends the posture experiment.
4. Deeper lowering remains planned.
5. Lowered lateral transfer remains planned.
6. Near-zero load / single support remains planned.
7. Foot lift, placement and repeated steps remain planned.

For lowering, select **P54 - centered lowering + return**, choose **10 mm** or
**20 mm** or **30 mm**, leave **Show the Gazebo simulation** checked, then click
**Run P54 lowering**. There is no push or force direction in this mode.
The normal full standing readiness precedes the motion. The GUI locks controls
while a job runs and shows standing, lowering, hold, rise/return and results.
Gazebo closes after the recording finishes. Plots and saved history remain.

The result displays measured lowering, final height offset, fresh-foot contact
evidence, a downloadable summary, and the height/load/speed plot. The run log
and `evidence/gui/<job-id>/` retain detailed diagnostics. Completed runs that
fail an audit show **Checks not met**, not **Passed**. Early exits remain errors.

## What changed for 20 mm

The maximum lowering is now 20 mm. The centered-COM IK, mass, geometry, motor
caps, PID gains, 25% minimum measured support share, force band and audit
thresholds are unchanged. No foot is lifted.

Lowering and rising each take `max(10, 1000 * lowering_metres)` simulated
seconds: 10 s each for 10 mm and 20 s each for 20 mm. Thus the complete motion
program is 37 or 57 simulated seconds after readiness. At target real-time
factor 0.25, allow roughly 4-5 or 5-6 wall minutes including startup.

The cached IK table uses intervals no larger than 0.25 mm: 41 poses at 10 mm,
81 at 20 mm. A geometry test caught that keeping only 41 poses for 20 mm
exceeded the sub-micrometre interpolation check. Refining the table fixed it;
the test tolerance was not increased. Analysis windows are derived from the
schedule rather than hard-coded to the earlier 37 s program.

## Files and responsibilities

| File | Addition |
|---|---|
| `src/my_robot_description/my_robot_description/motion/bilateral_lowering.py` | Up to 20 mm, slower deeper motion, finer cached poses and derived analysis windows |
| `tools/force_panel/config.py` | Strict 10/20 mm request validation; fixed lowering command with no push |
| `tools/force_panel/jobs.py` | Lowering progress, plot routing, completion messages and evidence-based roadmap |
| `tools/force_panel/static/index.html` | Roadmap, lowering mode and depth choice |
| `tools/force_panel/static/app.js` | Mode-specific controls, preview, results, history and roadmap selection |
| `tools/force_panel/static/app.css` | Roadmap layout and status styling |
| `tests/test_bilateral_lowering.py` | Extended geometry, speed and return checks for 20 mm |
| `tests/test_force_panel.py` | Lowering command/limits and roadmap evidence requirements |

The GUI remains an experiment client. It starts the existing guarded runner;
it does not implement a second motor controller. Future experiment additions
should follow this same path: planner, audit and tests, GUI mode, GUI-launched
measured run, saved evidence and then verified roadmap status.

## Verified GUI run - 1 November 2026

The 20 mm experiment was selected with the actual roadmap **Choose** button
and launched with **Run P54 lowering**, with Gazebo enabled. Saved job:
`20261101-173029-450121f3`, under `evidence/gui/`.

All motion, independent sensor, damping, measured lowering/pressure and live
force-guard audits passed. The GUI showed **Passed**, displayed the measured
lowering and final height offset, saved the plot, restored Run, and changed
the 20 mm roadmap milestone to **verified**. Package source hashes were
unchanged during measurement.

| Measurement | Result |
|---|---:|
| Requested / measured lowering | 20.000 / 20.006 mm |
| Maximum baseline-relative height tracking error | 0.0265 mm |
| Maximum final height offset | 0.0044 mm |
| Maximum final fore-aft offset | 0.0027 mm |
| Maximum load-share deviation from 50% | 0.0081 percentage points |
| Maximum vertical force error | 0.0422 N |
| Maximum force-weighted CoP / COM distance | 0.0226 mm |
| Maximum force sample gap | 0.023 s |

The build, JavaScript syntax check and 286 unit tests passed. The earlier
failing geometry-test log is preserved alongside the passing test log and
build log. Page, JS, CSS, API, summary and plot endpoints were checked after
completion. This is one GUI-launched measured 20 mm run, not a single-support
or walking result. Earlier passing 10 mm and lateral-transfer runs remain
separate evidence.

Next: extend centered lowering gradually toward the offline candidate, then
combine lowering with a measured lateral transfer. Each new experiment must
be added to this panel before calling its roadmap stage verified.

## 40 mm experiment

The next centered-posture stage raises the allowed lowering to 40 mm, with
an explicit GUI choice and its own roadmap entry. The 0.25 mm cached pose
spacing gives 161 poses. Lowering and rising each take 40 simulated seconds;
with baseline, hold and recovery, the program lasts 97 simulated seconds
after full standing readiness. No push, lateral unloading or foot lift is
introduced.

The physical URDF, motor caps and PID gains remain unchanged. Foot fixation,
COM centering, speed and static headroom were checked offline for the deeper
motion. All 287 tests, the build and JavaScript syntax validation passed.

Files changed: `bilateral_lowering.py` (40 mm envelope), panel `config.py`
(allowed request depth), `jobs.py` (roadmap stage), `index.html` / `app.js`
(option and display), `test_bilateral_lowering.py` (30/40 mm geometric checks)
and `test_force_panel.py` (40 mm command mapping). The frontend now also clears
an old connection-failure notice after a successful state refresh; previously
it could keep displaying that the panel was unreachable after recovery.

The 40 mm experiment is launched from the GUI with Gazebo enabled. A saved
passing result is required before its roadmap status becomes verified.

**Verified run:** `20261101-190242-c0747b90` was selected and launched through
the GUI with Gazebo enabled. It passed motion, sensor, damping, lowering and
live force-guard audits. Package source hashes remained unchanged throughout
measurement. Requested/measured lowering: **40.000 / 40.003 mm**; measured
fore/aft pelvis motion: **-16.908 mm**, versus **-16.905 mm** planned.

Maximum baseline-relative height tracking error was **0.0171 mm**, final
height offset **0.0024 mm**, final fore/aft offset **0.0026 mm**, force error
from weight **0.0489 N**, force-sample gap **0.023 s**, and peak measured
joint speed **0.02233 rad/s** (limit 0.05). No motor saturated. Both feet
stayed planted. These remain simulated measurements, not hardware accuracy
or a single-support demonstration.

The GUI saved the plot and summary and changed the 40 mm stage to verified.
HTTP checks for the page, JS, CSS, state API, summary and plot passed after
completion. Build and 287-test logs, source hashes, measurements and screenshots
are under `evidence/gui/20261101-190242-c0747b90/`. The login-started service
remains enabled; no experiment starts automatically.

The next useful milestone is a small lateral translation while holding this
validated lowered posture, with both feet still loaded. That will test the
combination of vertical and lateral posture planning before deeper unloading.

## 30 mm addition and timestamp correction

The GUI now offers a 30 mm milestone. The planner's maximum depth is 30 mm;
it caches 121 poses, lowers over 30 simulated seconds, holds for 6 s, rises
over 30 s and recovers for 8 s after a 3 s baseline. Total: 77 simulated
seconds after full readiness. Motor gains, physical geometry and acceptance
thresholds are unchanged. All earlier 10/20 mm controls remain available.

The first GUI run, `20261101-174038-45a9b5d6`, finished its motion and passed
the motion audit, but its sensor audit raised an exception. A pose received
inside the experiment window had a source timestamp just before the window.
Looking up the planned height at that negative relative time was invalid.
This failed run is preserved and was not used to verify the milestone.

`lateral_audit.py` and `lowering_audit.py` now use the baseline reference for
source times before program start, where the robot is still standing. Source
timestamps themselves are not changed, and freshness thresholds are not
relaxed. A regression test accepts a fresh pre-window pose but rejects a stale
one. All 287 tests passed after the correction.

The retry also exposed a browser form-validation issue: a completed lowering
job restores a zero force into the hidden push controls. The old force minimum
could prevent a repeat submission. `app.js` now disables those irrelevant
controls in lowering mode and re-enables them in push modes. Lowering uses
zero external force; the API still enforces each mode's allowed settings.

Files changed for this milestone: `bilateral_lowering.py` (depth limit),
`config.py` (allowed GUI depths), `jobs.py` (roadmap stage), `index.html`
(30 mm option), `app.js` (envelope and hidden-control validation),
`lateral_audit.py` / `lowering_audit.py` (pre-window reference),
`test_bilateral_lowering.py` / `test_lateral_shift.py` (geometry and timestamp
regressions), and this guide. The repeat is launched through the GUI as a
fresh job; its measurements determine the final milestone status.

The repeat, **`20261101-174923-9eb6266b`**, was launched through the GUI with
Gazebo enabled and **passed all audits**. The first failed job remains in
history. Requested/measured lowering: **30.000 / 30.004 mm**. Maximum
baseline-relative height tracking error: **0.0212 mm**; final height offset:
**0.0029 mm**; final fore/aft offset: **0.0019 mm**. Maximum left-share
deviation from 50% was **0.0070 percentage points**, force error from weight
**0.0614 N**, and force sample gap **0.024 s**. Source hashes stayed unchanged
during measurement. No foot was lifted, and the physical geometry, motor caps
and PID gains were not changed.

The GUI saved the response plot and summary and marked the 30 mm milestone
verified. Page, JS, CSS, state API and that run's summary/plot each returned
HTTP 200 afterward. Logs, screenshots, build and 287-test validation are in
`evidence/gui/20261101-174923-9eb6266b/`. The panel remains running and idle.
The next milestone should continue the measured lowering progression before
combining the lowered posture with lateral transfer.
