# Local force-experiment GUI

The force panel is a separate experiment tool. It selects disturbance parameters
and calls the existing runner. It selects predefined controller profiles and does not publish motor commands
or implement balance control.

## Open it

From the workspace root:

```bash
./scripts/start_force_panel.sh
```

Then open **http://127.0.0.1:8765** in a browser. The launcher sources ROS Jazzy
and the built workspace for you. No Internet account, extra frontend package
installation or external hosting is needed. For a fresh checkout, first build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select my_robot_description
```

If another application uses that port:

```bash
./scripts/start_force_panel.sh --port 8766
```

Only loopback is bound. Closing the browser tab does not stop the panel server.
Use **Close panel**, or Ctrl+C in its terminal, to stop it and close any active
owned experiment.

## What you can choose

| Setting | Current GUI input envelope |
| --- | --- |
| Direction | Forward +X or backward -X, in world coordinates |
| Magnitude | 0.1–100 N (default 2 N) |
| Pulse duration | 0.02–0.50 simulated second |
| Controller | Joint PID + IMU feedback, or joint PID only |
| Gazebo window | Show or run headless |

These are the existing effort-experiment input limits, not a guarantee that
every combination has been tested or will pass recovery. Both positive and
negative signs are generated from a direction plus a positive magnitude.
Backend validation also calls the existing `EffortPushConfig.validate()`;
bypassing a slider does not bypass its limits.

Sideways and vertical force directions, arbitrary application points, repeated
waveforms are not implemented in this panel. The
current robot has pitch-only leg joints and validated sagittal experiments.
The UI and job manager are separate from control, so future experiment profiles
can be added without putting GUI code into the motor controller.

## Stronger pushes

The expanded input envelope is for finding recovery limits, not a validated
stability range. Keep duration at 0.20 s and try 3, 5, 10, then 20 N, using a
fresh trial each time. Change one variable at a time. Both directions are
available. Larger force does not increase motor torque caps or controller gains.

The controller still stops at pelvis tilt >0.15 rad (about 8.6 degrees); the
recorder also retains its tilt/height guards. A strong push can therefore stop
the experiment before a complete tumble is recorded. Inspect the saved logs and
partial traces; a guard stop is not a completed recovery measurement. Sensor
failures remain distinct from physical posture-limit stops. The prior 2 N
results do not establish recovery at the newly allowed forces.

## Run and interpret an experiment

1. Select direction, newtons and duration. The arrow and requested impulse
   preview update immediately. The sketch is illustrative, not live telemetry.
2. Choose whether IMU balance feedback and the Gazebo window are enabled.
3. Press **Run experiment**. Settings lock while the captured request runs.
4. The runner starts a fresh no-push reference, then closes that world and starts
   a fresh push trial. The first simulation deliberately contains no push.
5. Each posture warms up and settles. After the push trial's baseline, the force
   is published, cleared after its configured simulated duration, and followed
   by a 10 s recovery observation.
6. The panel shows the result and the recorded reference/push plot. Download the
   full summary or inspect the evidence folder and run log.

At 0.25x pacing, a 15 simulated-second posture warmup takes about a minute of
wall-clock time. Allow roughly 4–5 minutes for a full paired experiment. Pulse
duration is simulated time; a requested 0.2 s pulse is approximately 0.8 wall
second at this pacing, plus transport timing uncertainty.

The nominal impulse preview is `magnitude * requested_duration`, in N s. It is
not a measured impulse. Published command windows, actual actuation timing and
calibrated force are distinct quantities, as in the existing experiment tools.

A completed run can have **checks not met**. That is different from a runner
error. Very small pushes may produce too little response for the comparison
criterion; inspect the summary rather than treating absence of motion as proof
of recovery. A process/sensor/clock error is shown separately as **Error**, with
its log retained. The panel does not hide failed runs or automatically retry.

## Stop and cleanup

**Stop & clear run** asks the existing runner to stop. Its recorder attempts to
clear the force in `finally`, and the runner closes the simulation it owns.
The panel waits for cleanup before allowing another run. A deadline fallback
terminates only processes carrying that particular panel job's ownership marker;
it does not issue a global Gazebo kill. A stopped or partial run is not labeled
as successful recovery.

The panel allows only one active experiment. Its runner uses ROS domain **58**
and a unique Gazebo partition, separate from the existing benchmark domain 57,
manual effort domain 56 and original position-mode domain 42. The command-line
runner retains domain 57 by default; only the panel supplies `--ros-domain-id 58`.
Other manually launched experiments should use their own domain/partition too.

## Where the evidence lives

- `evidence/gui/<timestamp-id>/`: the existing runner's reference/push data,
  controller terms, contact/IMU evidence, force handling, summary and response.png.
- `evidence/gui/.panel/<timestamp-id>.json`: request, job status and lifecycle.
- `evidence/gui/.panel/<timestamp-id>.log`: runner/plotting console output.

Each request gets a unique directory. History is loaded on panel restart. A job
left incomplete by an earlier server is marked **Interrupted**; a new server
does not pretend to own or resume that old process.

## Files and separation

| File | Responsibility |
| --- | --- |
| `scripts/start_force_panel.sh` | Source ROS/workspace and start the local app |
| `scripts/force_panel.py` | Small Python entry point |
| `tools/force_panel/config.py` | Validate UI parameters and construct an argument list for the existing runner |
| `tools/force_panel/jobs.py` | Own one subprocess job, save status/history, infer phases from files, cancel and generate plots |
| `tools/force_panel/server.py` | Loopback HTTP routes and serving the UI/artifacts |
| `tools/force_panel/static/index.html` | Accessible layout and illustrative robot sketch |
| `tools/force_panel/static/app.css` | Responsive presentation |
| `tools/force_panel/static/app.js` | Form/preview binding, requests, progress polling, results and history |
| `scripts/experiments/run_effort_push.py` | Adds an optional ROS domain argument; default 57 and existing behavior retained |
| `tests/test_force_panel.py` | Input limits, arguments, job outcomes, cancellation, duplicates, local request boundary and artifact-route checks |

The normal flow is:

```text
Browser form → local API → validated request → existing experiment runner
                                              ↓
                                  ROS/Gazebo + existing controller
                                              ↓
                            logs / summaries / response.png → browser
```

The browser never publishes force or torque directly. The existing recorder
publishes the disturbance; the existing posture node remains the motor-command
publisher. No controller/model files are changed for this panel. The UI polls
saved files, avoiding live parameter queries on the feedback node.

Only known static assets and a selected job's summary/plot are served. POST
requests require the local page token and local origin/host, and launch uses an
argument list rather than shell interpolation. There is no web option for
arbitrary commands or filesystem paths.


## Initial GUI validation (before the startup fix)

The full unit suite passed: 134 tests. A browser-started forward 2 N, 0.2 s
paired experiment completed with recovery checks passing and its recorded plot
visible in the panel (`evidence/gui/20261028-080444-c680adfc`). Its peak pitch
change was about 0.286 degrees. A separate real run was stopped from the browser
and its owned simulation processes exited cleanly. Settings/preview binding,
locked controls, history selection, restart persistence and restoring an active
request after browser reload were also checked. These simulation tests ran
headless; the new panel's Gazebo checkbox maps to the existing runner's `--gui`
flag, but a new visible Gazebo run was not part of this GUI validation.

Two attempted runs aborted at the existing joint-feedback timing guard during
startup (negative source age). One occurred on benchmark domain 57 and one on
the panel's dedicated domain 58, so domain isolation alone does not resolve it.
The no-push reference of the latter completed successfully before its push-world
startup failed. Both failures remain in history and their launch logs; there
are no automatic retries. If this happens, inspect the path in the panel log,
then the corresponding `reference/launch.log` or `push/launch.log` for the
underlying exception. That identified startup clock synchronization as the next robustness step.
The later fix is documented below; timestamp-age limits were not increased.

The forward completion above predates the GUI domain-58 isolation change.
See `evidence/gui/validation/validation.json` for subsequent test outcomes and
source-hash verification. All 19 tracked controller/model/configuration source
hashes match the preceding active-balance benchmark.

A subsequent browser-started backward **1.5 N × 0.12 s** paired run on domain
58 also passed (`evidence/gui/20261028-081650-c8cc1008`): peak pitch change
0.147 degrees, final return within bands, fresh contact evidence for both feet,
no saturated controller samples, and Gazebo queue/clear handling confirmed.
Its request values, both trial domain IDs, saved plot and browser result were
verified. This successful run does not erase the startup failures above.


## Startup clock fix

The startup race described above now has an explicit pre-activation clock gate.
The node waits for initialized simulation time and fresh joint timestamps,
while active timing limits remain unchanged. See [the detailed fix](../history/STARTUP_CLOCK_FIX.md)
for the cause, before/after code, changed files and validation. Historical
failed runs remain in history. Latest test results live in
`evidence/startup-clock/validation.json`.


## Live force arrow

New experiments automatically show an orange force-direction arrow in Gazebo
when its GUI is enabled. It is present only during the push command, so the
no-push reference has no arrow. See [Force arrow](../history/FORCE_ARROW.md) for appearance,
implementation, cleanup and timing limitations.


## Horizontal camera on startup

New Gazebo GUI windows now start in a level side view, including both the
reference and push worlds. The shared `make_simulator()` helper passes the
package's `config/gazebo_horizontal.config` via `--gui-config` whenever GUI
mode is requested. This configuration retains the installed Harmonic GUI
plugins (including force markers) and sets MinimalScene's camera pose to
`0 -1.5 0.32 0 0 1.5707963267948966` (x/y/z metres, roll/pitch/yaw radians).
Zero roll and pitch keep the view horizontal; yaw 90 degrees looks toward the
robot from the side. You can still move the camera manually within a run.
Headless launches have no GUI configuration argument. Existing open windows
are unaffected; the next experiment uses the new startup view.


## First-IMU discovery startup fix

The passive observer now allows up to ten wall seconds for its first valid IMU
message, then retains the original two-second stream-loss watchdog. This
prevents initial discovery delays being treated as broken established streams.
Active controller freshness checks are unchanged. See [IMU discovery fix](../history/IMU_DISCOVERY_FIX.md).

During pre-readiness settling, out-of-age joint samples are now discarded
without updating torque, integral state or wall-watchdog time. Fresh data
restarts the full settling window. After readiness the original timing
violation still stops the run. See the joint-startup section of the same guide.

## Visual-only simulation

Choose **Visual only** in **Run mode**. This always opens Gazebo, uses the
selected direction, magnitude, pulse duration and balance setting, and starts
one world. It applies the pulse immediately after the visual standing check passes,
with no extra countdown. Visual mode requires 0.5 simulated seconds within
0.03 rad joint error and 0.05 rad/s joint speed; when balance is enabled it
also waits for the existing 1-second IMU correction ramp and contact gate.
This replaces the measured PID profile’s 15-second minimum and stricter
settling check only in visual mode. Pulse duration still uses simulation time. It clears the force after the requested
simulated duration and keeps Gazebo open until **Stop & clear run**. To try
another force, stop, change the settings, and start again.

There is no reference world, baseline recorder, response plot or recovery verdict.
Standing warmup and existing controller posture/sensor guards remain enabled.
In visual mode only, a controller/observer exit after readiness does not close
Gazebo. The posture controller publishes zero motor commands on exit; physics
continues so you can observe the fall. This is passive motion after control
stops, not an attempt to recover from the ground. Measured experiments still
shut down on these exits. Startup failures still prevent a visual push. Launch logs and command events are saved
in the run folder. Visual feedback is not a calibrated force measurement.

## Keep the server running in the background

If the browser reports connection refused, the local server may be stopped.
On this Linux desktop run `./scripts/start_force_panel_background.sh` from the
workspace. It starts `biped-force-panel.service` as a transient systemd user
service and reuses it if already active. It survives closing the launching
terminal or chat. It does not enable automatic start after reboot/logout.
Close panel deliberately stops the service; run the script again to reopen.
The server remains on http://127.0.0.1:8765/ with its existing loopback boundary.

## Live center of mass in visual mode

New visual-only worlds show cyan whole-body COM, yellow ground projection and
trail, white sole outlines, and a green contact-qualified flat-sole hull. The
large yellow ring is a locator, not a stability margin. Observations are saved
in `com.jsonl`. See `docs/guides/CENTER_OF_MASS_VISUAL.md` for calculations, code roles
and plotting instructions. COM tracking continues after a controller guard stop
while Gazebo remains open. Refresh the panel and start a new visual run.

## Experimental stronger ankle response

The Balance controller selector offers the original profile and **stronger ankle
response (experimental)**. This raises only the IMU correction allowance from
0.5 to 1.5 N m per ankle; the total 6 N m ankle motor cap stays unchanged.
Four full measured 25 N, 0.20 s candidate trials passed, two per direction.
See `docs/history/BALANCE_CAP_IMPROVEMENT.md` for the comparison and limitations.

Visual mode has a **Use full measured settling (slower)** checkbox. The
experimental selection recommends it because fast visual warmup produced a
different 25 N outcome. Uncheck it for fast demonstrations. Full visual settling
keeps the world open after a controller exit, and does not issue a measured
recovery verdict. The original profile remains the default.

## Damped ankle profile (29 October 2026)

The optional **damped ankle response** uses a 1.5 N·m per-ankle IMU correction
cap and total pitch-rate gain 8, compared with gain 4 in **stronger ankle
response**. Motor limits remain unchanged. Four measured ±25 N / 0.20 s pushes
recovered; the new profile reduced rebound. Small backward residual tilt took
slightly longer to settle. Original defaults remain available and unchanged.

Selecting an experimental profile enables **Use full measured settling
(slower)** for visual comparisons. Keep it checked at 25 N to match measured
warmup. The fast visual option can behave differently. See
[BODY_DAMPING_IMPROVEMENT.md](../history/BODY_DAMPING_IMPROVEMENT.md) for calculations,
file changes, exact results and reproduction instructions.
