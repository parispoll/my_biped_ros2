# Force arrow inside Gazebo

Every new push experiment now requests an orange 3D arrow in the Gazebo GUI.
Enable **Show the Gazebo simulation** in the force panel and run normally.
The first world is the zero-force reference and deliberately has no arrow.
In the second world, the arrow appears during the published push command and
is removed when that command is cleared. A 0.30 simulated-second duration at
the panel's 0.25× pacing gives about 1.2 wall seconds to see it.

The arrow points along the world X force direction: +X for forward, -X for
backward. It moves with the recorded model position, ending just outside the
pelvis so the head is visible. Its drawing offset is not a change to the force's
application point. The force still targets the existing model/canonical link.

## What changed

- `src/my_robot_description/my_robot_description/force_visual.py` contains the
  visual geometry, owned marker IDs, transport worker, cleanup and status.
- `scripts/push_experiment.py` calls `update(force_x, body)` after publishing the
  push, refreshes its position while active, calls `hide()` after publishing
  clear, and calls `close()` in cleanup. Both effort and position experiments
  use this existing shared recorder.
- `tools/force_panel/static/index.html` adds a short note explaining the Gazebo arrow.
- `tests/test_force_visual.py` verifies both signs, head orientation, magnitude
  scaling, owned-only cleanup, missing renderer behavior, and nonblocking force
  updates while a simulated GUI request is stalled.

The arrow uses a box shaft plus cone head. Although the message enum includes
`ARROW`, the installed Harmonic GUI implementation does not render that type;
box and cone are supported. See the official
[Gazebo GUI 8 MarkerManager source](https://github.com/gazebosim/gz-gui/blob/gz-gui8/src/plugins/marker_manager/MarkerManager.cc).
Its existing `/marker_array` service accepts visual markers. No world, URDF,
physics plugin, controller, gain, force limit or force publication is changed.

```text
Recorder publishes persistent wrench → updates visual mailbox
                                         ↓ separate worker
                                  GUI /marker_array service

Recorder publishes clear → clears mailbox → delete owned arrow markers
```

## Meaning and limitations

This is an illustration of the **command window**, not a measured force vector
or a precise indication of Gazebo's actuation time. Renderer and transport
latency can shift its appearance slightly. It is not feedback to the controller.

The requested magnitude controls the schematic total length:
`min(0.55, 0.20 + 0.10 * abs(force_x))` metres. The minimum size and upper clamp
make it readable; this is not a calibrated Newton-to-length measurement.

Requests run in a separate worker with a latest-value mailbox and bounded
request timeout. The main force loop does not wait for a renderer. A headless
simulation without `/marker_array` continues normally. A marker failure is
reported under `force_visualization` without invalidating physics measurements.
A service acknowledgement means the GUI accepted the request; it does not prove
the frame was rendered or calibrate the force.

Each recorder owns a unique namespace and only deletes its own two markers.
No `DELETE_ALL` is used. Markers also have a 0.2 simulated-second lifetime,
refreshed during the pulse, to expire a stranded arrow after a recorder crash.
On interrupt, the existing force-clear action is requested first, then visual
cleanup. Visual removal never substitutes for physical force clearing.

The full unit suite passed: **147 tests**. Actual Gazebo validation artifacts,
including the GUI's own screenshot, are under `evidence/force-arrow/`. The first
attempt stopped at the existing observer's startup IMU watchdog before reaching
the experiment; its failure is retained in `validation-attempt-01.json` and
`paired-backward-01/`. The second paired attempt passed its reference but stopped at the existing joint-feedback
watchdog (83 ms source age) before its push; see `validation-attempt-02.json`.
The focused recorder test and final aggregate outcome are in `recorder-validation.json`
and `validation.json`. Timing limits were not relaxed for this visual feature.


## Rendered verification

The focused live effort recorder test completed with a -2 N pulse lasting
0.30 simulated seconds and its recovery checks passing. Gazebo's marker-list
service confirmed no arrow during baseline, both owned arrow parts during the
pulse, and their removal after clear. The GUI accepted 35 visual updates and
the clear request, with zero visualization errors. Its own screenshot service
captured the backward arrow in `evidence/force-arrow/force-arrow.png`, which was
visually inspected. This is a focused recorder/renderer check, not a passing
full paired benchmark. All tracked controller/model hashes match the preceding
startup-fix snapshot.
