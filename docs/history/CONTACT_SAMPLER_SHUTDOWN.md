# Fixing the contact sampler's shutdown traceback

## What the log showed

In GUI run `20261101-193044-96a64bc9`, the complete motion and its measurement
audits passed. After `[shift:done]`, the runner sent SIGINT to close the launch.
`sample_contacts.py` then raised a conversion error inside ROS's
`_take_subscription` / `take_message` path and exited with code 1.

The timing and call path are consistent with a subscription being taken while
the default ROS signal handler tears down the context. The fix removes that
shutdown ordering hazard; it does not catch and hide the conversion error.

## Before

```python
rclpy.init()
node = ContactSampler()
rclpy.spin(node)
# Default ROS signal handling may shut down the context while spin is active.
```

## After

The sampler installs its own SIGINT/SIGTERM handler. That handler only sets a
stop flag. ROS is initialized with `SignalHandlerOptions.NO`, so the default
ROS signal handler cannot shut its context down in the middle of a callback.
An explicit single-thread executor processes callbacks with a 20 ms maximum
wait, checking the flag between iterations.

Cleanup order:

```text
SIGINT / SIGTERM -> set stop flag
                -> finish current callback
                -> cancel forwarding timer
                -> stop executor
                -> destroy node
                -> shut down ROS context
                -> restore previous signal handlers
```

Unexpected callback exceptions still propagate. Message contents, source
timestamps, topics, QoS, 50 Hz forwarding timer and use of simulation time
are unchanged. Robot geometry, gains and motion limits are unchanged.

## Files changed

| File | Purpose |
|---|---|
| `src/my_robot_description/scripts/sample_contacts.py` | Explicit signal request and ordered executor/node/context cleanup |
| `tests/test_contact_sampler_lifecycle.py` | Verify callback completion before context shutdown and propagation of real runtime errors |
| `scripts/check_contact_sampler_shutdown.py` | Exercise actual ROS message forwarding during repeated process signals |
| This guide | Explain the failure, change and evidence |

## GUI startup issue found during verification

The first GUI baseline attempt on 4 November,
`20261104-074844-799b4ab8`, stopped before the sampler was started. Gazebo's
Qt window reported `could not connect to display`; controller activation
subsequently timed out. That failed run remains preserved.

The login-started panel process had no DISPLAY, WAYLAND_DISPLAY or XAUTHORITY
environment values, although the user service manager had valid desktop
settings by the time Run was clicked. Starting the panel before the graphical
desktop imported those values left its child-process environment stale.

`tools/force_panel/jobs.py` now refreshes only those three display-routing
variables before launching a GUI experiment. Other service-manager variables
are not imported, and the owned-job marker is preserved. If the manager is
unavailable, the existing environment is retained. The new regression test in
`tests/test_force_panel.py` verifies this filtering and fallback.

## Validation

The package build and **291 unit tests passed**. The real ROS traffic check
in `evidence/contact-sampler-shutdown-01/` passed six runs: three SIGINT and
three SIGTERM. Each run forwarded both feet before the signal and continued
receiving raw contact traffic during shutdown. All six exited with code 0;
there were no tracebacks or source-timestamp errors.

This traffic check uses synthetic `Contacts` messages to test the ROS lifecycle;
it is not a contact-force validation experiment. The existing unit check also
confirms forwarding preserves the original message and timestamp.

Reproduce it in a ROS domain separate from any live experiment:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=147
python3 scripts/check_contact_sampler_shutdown.py \
  --output evidence/my-contact-shutdown-check-01
```

Use a fresh output folder. End-to-end Gazebo validation is a separate 10 mm
baseline run launched through the normal GUI; its summary and launch log
determine motion-audit success and whether the sampler exits cleanly.

## Completed end-to-end result - 4 November 2026

GUI retry **`20261104-075355-98ebdd79`**, with Gazebo enabled, completed the
10 mm baseline and passed every motion, sensor, damping, lowering and force
guard audit. Its launch log reports `[shift:done]`, then SIGINT to the sampler,
then **`process has finished cleanly`** for `sample_contacts.py`.

There were no tracebacks anywhere in that launch log. Source hashes stayed
unchanged during the run. Comparing package hashes with the user's earlier
rightward transfer confirms the sampler is the only changed package source;
the geometry and controller sources match. The panel display fix lives in
the separate GUI tools directory.

The final suite passed **292 tests**; the build passed. Evidence is in
`evidence/contact-sampler-shutdown-01/` (six real ROS signal tests) and
`evidence/gui/20261104-075355-98ebdd79/` (GUI run, logs, plots, test/build logs
and `shutdown-verification.json`). One clean Gazebo shutdown plus six isolated
signal tests verifies these cases; it is not an exhaustive concurrency proof.
