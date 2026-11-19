# Live whole-body center of mass

Visual-only runs now show whole-body COM during standing, the push, recovery and passive falling after the controller exits. Start a new run from the force panel; existing worlds do not gain a new plugin automatically.

## Read the display

| Visual | Meaning |
| --- | --- |
| Cyan sphere and crossbar | Whole-body COM in world coordinates; sphere size and crossbar length are illustrative |
| Yellow dot | Vertical projection of COM onto the flat ground, drawn slightly above the surface to avoid flicker |
| Yellow locator ring | Centered on that same projection; its 12 cm radius is only for visibility, not a stability margin |
| Yellow line and short trail | Projection connection and recent ground-projection path (up to five simulated seconds) |
| White outlines | Current sole geometry |
| Green outline | Convex hull of flat, near-ground soles with fresh ground-contact evidence |

The green outline disappears when contact evidence is stale/unknown or the sole is lifted or tilted. With one qualifying foot it includes only that foot. This is a geometric approximation, not a pressure-derived contact patch, center of pressure or ZMP. A COM projection inside the hull alone does not prove dynamic stability: the robot can still have enough motion to continue tipping.

The true COM can be partly hidden by the robot from a side view. The crossbar and enlarged marker help locate it; orbit the camera for the clearest ground view. All marker sizes are drawings, with no collision, mass or force. Stale COM telemetry hides the markers rather than leaving a confident-looking frozen display.

## The calculation, step by step

For each physical robot link i, take its mass m_i and its local inertial-origin offset. Transform that offset by the link's current world pose to obtain its actual world COM position r_i. Then:

`whole_body_COM = sum(m_i * r_i) / sum(m_i)`

For this model the total mass is:

`7.5 + 2 * (1.4 + 1.0 + 0.9) = 14.1 kg`.

The torso's COM is 2 cm above its link origin. The calculation includes this offset and the masses of both feet. It does not assume the pelvis stays upright, that the two legs have equal angles, or that the feet remain flat. All masses and link poses are read from Gazebo's physical model at the same PostUpdate, which avoids mixing a current base pose with old joint positions.

The projection is `(COM_x, COM_y, 0)` because this world has a flat ground plane at world z=0. This ground-plane assumption must be revisited for slopes or uneven terrain. Marker drawing uses small positive heights to avoid depth flicker.

## Separate responsibilities

```mermaid
flowchart LR
  A[Gazebo link world poses and inertias] --> B[Read-only ComProbe]
  B --> C[/biped/com_state]
  C --> D[Python COM observer]
  E[Foot collision messages] --> D
  D --> F[Timestamp selection and geometric hull]
  F --> G[Marker mailbox and GUI worker]
  G --> H[Gazebo markers]
  D --> I[com.jsonl]
  I --> J[COM response plot]
```

Nothing here publishes motor commands. Controller gains, torque caps and guard thresholds are unchanged. The observer continues after a visual-mode controller exit because it uses Gazebo link poses directly, rather than requiring a live controller to supply joint positions.

## Files

| File | Responsibility |
| --- | --- |
| `src/my_robot_description/plugins/com_probe.cpp` | Reads the robot's physical links, transforms inertial origins and sole corners, publishes a same-update COM sample at about 50 Hz simulated time |
| `src/my_robot_description/my_robot_description/sensing/com_geometry.py` | Validates telemetry, selects contact evidence at/before the COM timestamp, computes contact-qualified geometric hulls |
| `src/my_robot_description/my_robot_description/visualization/com_markers.py` | Builds only the owned COM marker IDs and cleanup messages |
| `src/my_robot_description/my_robot_description/visualization/com_visual.py` | Separate GUI-service worker, stale-display timeout and marker cleanup |
| `src/my_robot_description/nodes/sensing/show_center_of_mass.py` | Combines COM and contact observations, records `com.jsonl`, updates display |
| `src/my_robot_description/launch/effort_standing.launch.py` | Optional `com_visual`/`com_recording` launch arguments; injects the read-only plugin and starts the optional display node |
| `scripts/experiments/run_visual_push.py` | Automatically enables COM in new visual runs, saves isolated session metadata and the COM recording |
| `scripts/plots/plot_com_response.py` | Plots COM fore/aft and height changes during a saved visual push |
| `tests/test_com_geometry.py`, `tests/test_com_markers.py` | Verify schema validation, future/stale/missing/tilted/one-foot contact behavior, marker geometry and owned cleanup |

The CMake file builds/installs the observer and display node. Panel HTML/JavaScript adds the color legend. Existing measured runners do not enable the new observer by default; prior measured-study source snapshots remain historical. No stronger-force recovery improvement is claimed from adding this instrumentation.

## Use it

Refresh http://127.0.0.1:8765/, choose **Visual only**, select a force and duration, and start a fresh simulation. The COM display starts during warmup and continues until Stop. Keep the initial force modest while learning the display.

To plot any new visual run:

```bash
python3 scripts/plots/plot_com_response.py evidence/gui/YOUR-RUN-ID
```

This writes `com-response.png` and `com-response.json` beside `com.jsonl`. It uses a pre-push baseline and plots fore/aft movement relative to the current foot centers. It is an observation plot, not a recovery pass/fail test.

For a manual launch, `com_visual:=true` enables the optional plugin/display, and `com_recording:=/absolute/path/com.jsonl` records the observations. The destination parent must already exist; an existing recording is never overwritten.

## Verified examples

- Build succeeded and 195 tests passed.
- A 10 N, 0.20 s visual push/recovery recording showed about 1.60 cm of fore/aft COM displacement relative to its pre-push baseline and current foot centers (`evidence/com-visual-03/com-response.png`). This fast visual run is not a replacement for the full measured recovery protocol.
- An independent URDF forward-kinematics calculation matched the COM output within 0.1 mm; the settled and near-settled checks were substantially closer. This validates internal coordinate/mass consistency, not real-world hardware mass identification.
- The Gazebo marker-list query did not provide usable replies in the validation client. That failed diagnostic is preserved in the per-attempt validation files. Marker delivery was instead confirmed by successful observer service acknowledgements and inspected rendered screenshots.
- A 25 N visual push stopped the posture controller, but COM observations continued for more than two simulated seconds afterward. The fallen COM height was about 0.084 m (`evidence/com-visual-fall-01/validation.json`); the cyan marker and yellow projection ring remained visible in the saved Gazebo screenshot. All 1,572 COM packets in that test were valid, with zero display-worker errors.

The useful next comparison is COM position and motion together with ankle torque saturation during the same pulse. That evidence can guide a controller change before we add a stepping response.
