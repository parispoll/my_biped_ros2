# Physical lateral pulse and the restored experiment page

This milestone tests a real Gazebo external-wrench command on the P54-class robot while both feet remain planted. It also restores the loopback experiment server and adds a page mode for the current ten-joint model.

## 1. What was wrong with the page

The server on port 8765 was stopped. Its old controls only launched the earlier six-joint sagittal experiments. Starting that page alone would not have exposed the new P54 load-feedback controller.

The server now runs as the user's background `biped-force-panel.service`, started with:

```bash
bash scripts/start_force_panel_background.sh
```

Open <http://127.0.0.1:8765/>. It remains available after the chat, until stopped or the user session/service ends. After a reboot, use the start command again. `Close panel` intentionally stops the server and any experiment it owns.

The P54 mode is the page default. Legacy six-joint measured and visual-only modes remain separately labelled and available. Their larger X-direction force range is not the current P54 test envelope.

## 2. Controls for the current robot

Select **P54 · measured load transfer + lateral pulse**.

- **Loaded side:** chooses the foot that will carry approximately 70% of vertical support during the hold.
- **Direction:** independently chooses positive world Y (robot left) or negative world Y (robot right) for the external force.
- **Force:** 0–1 N for this initial bilateral disturbance test. Choose 0 N for a no-push reference.
- **Duration:** 0.02–0.2 simulated seconds.
- **Controller:** measured load-share feedback on, or joint PID only.
- **Show the Gazebo simulation:** opens the native Gazebo view.

Click **Run P54 transfer**. The settings lock while one owned job runs; **Stop & clear run** requests force clearing and simulator cleanup. The page shows phase progress, results, downloadable summary, a measured response plot and saved history.

The test uses full measured standing readiness, then the existing 37-simulated-second transfer program. The pulse starts 13 simulated seconds after readiness, two seconds into the hold. At real-time factor 0.25, that is about 52 wall seconds after readiness, plus startup/settling. Allow roughly four wall minutes for the complete experiment.

The orange force arrow is visible during the command window. Cyan shows COM and yellow its ground projection/trail. The P54 camera is now level and front-facing so world-Y motion is visible. Six-joint sagittal experiments retain the original level side camera.

## 3. Applying and clearing an actual external force

`my_robot_description/lateral_pulse.py` advertises the existing Gazebo persistent-wrench and clear topics in the experiment's unique Gazebo partition. The force vector has only its world-Y component set; the target is model `my_biped` through the existing ApplyLinkWrench system.

The coordinator reads the latest complete controller record to schedule publication against program time. It does not spawn a new ROS recorder after readiness. Once the command window reaches the requested duration, it publishes the clear command. Cleanup attempts a clear again before stopping an owned live simulation.

Debug receipts must show both `Queueing persistent wrench` and `Clearing persistent wrench for entity`. The recorded command window must fall between the requested duration and 0.03 s beyond it. Transport delay and exact physical force timing/magnitude are not independently calibrated.

The force arrow uses a separate marker worker, so waiting for the GUI never delays force publication or clearing. The new Y-arrow helper rotates the existing box/cone illustration into the lateral direction. Its size is schematic, not a physical scale.

## 4. First result: capacity is not damping

The first 1 N / 0.2 s pulse trial, `evidence/gui/20261030-133138-6780a1af`, queued and cleared the force. It subsequently developed recovery oscillations and the existing force-sum guard stopped the experiment. It remains a failed run. The prior no-push reference, `20261030-132637-bd2ed48a`, passed.

The failed run's guard observed a total vertical reaction around 300.8 N, outside the permitted 5% band around 218.5668 N. We did not raise that band or treat a successful force publication as a successful recovery.

The new actuators provided torque capacity, but the roll-joint velocity damping had still been inherited from the much lighter prototype. We introduced a P54-specific candidate change:

```python
# Earlier: all profiles used this roll velocity gain
KD = [.1, .1, .15, .05, .1] * 2

# Now: stronger roll velocity damping for P54 only
KD = ([2., .1, .15, .05, 2.] if ACTIVE['name'] == 'robotis_p54'
      else [.1, .1, .15, .05, .1]) * 2
```

Joint order per leg is hip roll, hip pitch, knee pitch, ankle pitch, ankle roll. Units are Nm·s/rad. The derivative torque is `tau_d = −KD × joint_velocity`; its instantaneous contribution `tau_d × joint_velocity = −KD × joint_velocity²` opposes motion.

This changes motor feedback damping, not the model's passive friction or actuator ratings. Proportional/integral gains, mass/inertia, the slow outer load-share loop and acceptance thresholds remain unchanged. Legacy roll gains remain 0.1. A fresh reference and pulse trial are required for the revised controller; the old reference is not reused for a matched response comparison.

## 5. Code changes

| File | Change |
|---|---|
| `my_robot_description/lateral_pulse.py` | Scheduled small world-Y wrench, command events, queue/clear receipts and cleanup |
| `my_robot_description/force_visual.py` | Lateral arrow geometry and optional axis selection, preserving X-arrow behavior |
| `my_robot_description/roll_posture.py` | Explicit P54-only roll velocity damping |
| `my_robot_description/simulation.py`, `config/gazebo_front.config` | Selectable level front camera; default side view preserved for older callers |
| `launch/roll_standing.launch.py` | Front camera for ten-joint trials; optional wrench debug receipts |
| Workspace `scripts/run_lateral_shift.py` | Pulse options, proof of Gazebo handling and complete-result reporting |
| `tools/force_panel/config.py` | P54 request validation and fixed runner command, including independent loaded-side/force direction |
| `tools/force_panel/jobs.py` | P54 phase progress, result handling and plotting |
| `tools/force_panel/static/index.html`, `app.js` | Current-model controls, model-specific limits/labels and measured results |
| Workspace `scripts/plot_lateral_experiment.py` | Panel plot of COM, measured foot shares and raw IMU, with command-window shading |
| Workspace `scripts/compare_lateral_pulse.py` | Matched reference/pulse raw-IMU comparison with model, gain and source checks |
| Tests | Profile/command limits, partial JSON reads, lateral arrow direction, completed failed-check jobs and explicit damping selection |

Package-relative paths above are under `src/my_robot_description/`. Physical model sources and saved metadata identify the controller version for each trial.

## 6. What a passing test would establish

We require all existing posture, joint speed, COM, contact, force, sensor freshness, damping and return checks to pass, as well as Gazebo queue/clear evidence. The raw lateral IMU response is compared with a fresh 0 N reference aligned by program time; the incremental response must exceed a 0.005 m/s² detection floor.

For GUI trials, marker-service acknowledgements show that arrow updates and deletion were accepted. A passing result concerns this small pulse and this bilateral trajectory. It does not establish larger-force limits, walking, single support, hardware calibration, or that feedback-on outperforms feedback-off under every push.

## 7. CLI reproduction

After sourcing ROS and the built workspace:

```bash
python3 scripts/run_lateral_shift.py --gui \
  --actuator-profile robotis_p54 --unload-fraction 0.3 --direction left \
  --load-feedback --pulse-force-y 1 --pulse-duration 0.2 \
  --output evidence/my-p54-lateral-pulse-01
```

Here `--direction` is the loaded side; the signed `--pulse-force-y` independently sets force direction. Use fresh output folders. For a reference, omit the pulse force or set it to zero. The page runs these same commands and saves outputs under `evidence/gui/`.

## 8. Verified outcomes

The build and **283 unit tests passed**. Both revised runs were launched through the actual page:

| Run | Job ID | Result |
|---|---|---|
| Fresh 0 N reference, revised damping | `20261030-133858-609cf1df` | All checks passed |
| +1 N, 0.2 s pulse, front-view GUI | `20261030-134412-6b33192c` | All checks passed |

The pulse's recorded command window was **0.204 s**. Gazebo confirmed queueing and clearing. The GUI accepted **23 arrow updates** and its deletion with no marker errors.

The incremental lateral IMU response versus the matched fresh reference peaked at **0.07598 m/s²**, above the predefined 0.005 m/s² detection floor. Final COM excursion was **0.0353 mm**; final pelvis excursion was **0.0551 mm**. Both feet remained grounded, and no motor saturated.

The page, JavaScript, CSS, state API, summary download and PNG endpoint each returned HTTP 200 after completion. The page visibly showed Passed, restored the Run button, and served the saved plot. Legacy visual controls were checked and the tab was returned to P54 mode. The server remains running in the background with no active experiment.

Evidence: `evidence/p54-physical-pulse-01/` contains the matched response plot/report, build/test logs, server health results, verification hashes and page screenshots. The initial failed pulse is retained in history. This is one verified +Y pulse while the left foot is loaded; other directions/settings remain experiments guarded by the same checks.

## 9. Opposite-direction check — 1 November 2026

We kept the left-loaded 70/30 posture and changed only the external force from
+1 N to −1 N, retaining the 0.2 s request, controller gains and audit limits.
This isolates push direction; it does not yet test the mirrored right-loaded posture.

The first attempt, `evidence/p54-negative-pulse-01`, completed and returned
successfully, but failed the overall audit because of recording coverage gaps:
the largest raw IMU gap was 0.09 s and the foot-force gap was 0.07 s. Its failed
summary remains unchanged. Physical recovery alone is insufficient to pass.

The identical repeat, `evidence/p54-negative-pulse-02`, passed every audit,
including sensor coverage, measured foot loads, contact, posture, motor limits,
load feedback and return. Gazebo confirmed queueing and clearing; the recorded
command window was 0.204 s. Comparison with the source/gain-matched zero-force
reference detected an incremental lateral acceleration of 0.07483 m/s², above
the existing 0.005 m/s² threshold. No controller or simulator source was changed.

This repeat was headless: it validates dynamics and recorded plots, and does not
add a new GUI arrow observation. The earlier +Y GUI proof remains separate.
The panel was restarted and its page, JS, CSS, state API, saved summary and plot
endpoints each returned HTTP 200. CLI runs are saved in their evidence folders;
they are not automatically added to panel-managed job history.

Reproduce through the panel with **loaded side Left**, **direction Right**,
**force 1 N**, **duration 0.2 s**, and **measured load-share feedback** enabled.
Check **Show the Gazebo simulation** to watch it. The force still occurs 13
simulated seconds after standing readiness.

CLI equivalent (after sourcing ROS and the workspace):

```bash
python3 scripts/run_lateral_shift.py \
  --actuator-profile robotis_p54 --unload-fraction 0.3 --direction left \
  --load-feedback --pulse-force-y -1 --pulse-duration 0.2 \
  --output evidence/my-negative-pulse-01
```

Add `--gui` for Gazebo. Use a fresh output folder. The measured response plot
and matched comparison are in `evidence/p54-negative-pulse-02/response.png` and
`evidence/p54-negative-pulse-comparison-01/comparison.png` respectively.
The next controlled test is the mirrored right-loaded posture, before increasing
the force or attempting foot lift.
