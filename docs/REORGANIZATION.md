# Code organization and educational cleanup — 14 November 2026

The goal of this change is to make the verified robot understandable before adding new behavior. Geometry, motor caps, controller parameter values and planned motions are unchanged.

## What changed

The previous flat Python library and command-line script directory mixed unrelated responsibilities. Source files now live beside files that do the same kind of work. Every Python file has a module explanation; the main motion functions have unit/contract docstrings, and critical PID/anti-windup/support-exchange calculations have explanatory comments. Dense one-line Python was expanded with AST-checked formatting.

| Responsibility | Current location |
|---|---|
| Physical robot and kinematics | `src/my_robot_description/my_robot_description/model/` |
| Planned lowering, transfer, lift, steps and walking | `src/my_robot_description/my_robot_description/motion/` |
| Joint PID, integration and timing | `src/my_robot_description/my_robot_description/control/` |
| Decode and interpret measurements | `src/my_robot_description/my_robot_description/sensing/` |
| Independent saved-data checks | `src/my_robot_description/my_robot_description/validation/` |
| COM/force markers | `src/my_robot_description/my_robot_description/visualization/` |
| Launch preparation and dependency ordering | `src/my_robot_description/my_robot_description/runtime/` |
| ROS executable adapters | `src/my_robot_description/nodes/control/`, `sensing/`, `legacy/` |
| Gazebo C++ plugins | `src/my_robot_description/plugins/` |
| CLI tools | `scripts/experiments/`, `planning/`, `checks/`, `plots/`, `analysis/`, `studies/`, `guides/`, `services/` |
| Current learning material | `docs/learning/` |
| Current references and experiment guides | `docs/guides/` |
| Historical research and frozen handoffs | `docs/history/` |

Every source/tool group has a local README file map. The complete old→new path list is [path-migration.json](../evidence/preparation/code-organization/path-migration.json).

## What was removed as redundant or generated

- Duplicate contact-coverage, joint-command replay and body-sensor checks in the first-step and walking audits. Both now use `validation/motion_audit.py`, whose functions return named immutable evidence records.
- Duplicate imports introduced by direct script startup support.
- Repeated CMake plugin setup/install rules. One helper retains each plugin's original linked libraries.
- Obsolete flat source locations: files were moved, not copied into duplicate wrappers.
- Dangling installed Python symlinks pointing at the removed flat module locations.
- Generated Python cache folders and empty old source directories.

See [the cleanup manifest](../evidence/preparation/code-organization/cleanup.json) for exact generated paths removed. Build/install directories remain because ROS needs them. VS Code hides generated build/cache folders and excludes large evidence folders from ordinary text search; measurements remain accessible in the explorer and GUI.

Older study code remains in clearly named folders because historical GUI modes, tests and calculations still use it. Old reports and experiment evidence were not deleted or relabelled as new results.

## Stable entry points and changed imports

The existing panel service still starts `scripts/start_force_panel.sh` through the same background helper. Installed ROS executable basenames are unchanged, so launch files still call `hold_roll_stance.py`, `show_center_of_mass.py` and the other registered nodes.

Python imports are explicit, for example:

```python
from my_robot_description.motion.alternating_walk import AlternatingWalkProgram
from my_robot_description.control.roll_posture import RollPosture
from my_robot_description.validation.alternating_walk_audit import audit_alternating_walk
```

Old commands such as `python3 scripts/run_lateral_shift.py` are now:

```bash
python3 -m scripts.experiments.run_lateral_shift --help
```

Direct execution of the new grouped file path also works. There are no old-name Python re-export wrappers to maintain.

## Verification and practical limits

- `colcon build --symlink-install --packages-select my_robot_description` passed.
- ROS launch arguments loaded through the reorganized library and retained installed executable names.
- The current suite contains **335 tests**. The first complete sweep passed its substantive checks except for three obsolete script import references and the startup tests' sandbox log path. Those imports were corrected; the affected modules and startup checks passed reruns with writable ROS logs. GUI, re-audit routing and node lifecycle/capture checks also passed targeted final reruns. Logs are retained in `evidence/preparation/code-organization/`.
- Source review verified identical calculation ASTs for **62 Python source files**, ignoring moved imports and documentation. The three reviewed exceptions are the deeper actuator-config lookup and the two audits delegating identical logic to a shared module.
- All **32 physical/configuration files** retain their original values or implementation. C++ additions are explanatory headers only; YAML additions are comments only.
- Replaying the unchanged successful walking recording `20261113-181745-9d3ab96d` passed all six audits. Every returned report field matches its original report; numerical comparison tolerance is `1e-12`. The original summary remained unchanged.
- A new **offline** walking preview completed through the GUI using the new script paths: `20261114-095227-0b3c6bbb`. It checked the full geometry/static path and generated its plot without starting Gazebo or publishing motor commands.

No new measured walking simulation was needed for an organization-only change. The replay and source comparison establish preservation of calculations and recorded evidence; they are not a claim of a new physical experiment under the reorganized source version.

The walking plan's canonical complete phase windows remain `motion_plan.steps`; the inherited `schedule` field covers the initial first-step prefix. This existing metadata convention is explained in the walking guide.

## Historical provenance and recovery

The exact pre-change source, scripts, tests and documents are preserved in [before-source.tar.gz](../evidence/preparation/code-organization/before-source.tar.gz). Historical measurement folders and their recorded source hashes were not rewritten. The frozen technical handoff and its snapshot also retain their original contents.

Reorganization changes source paths and comments, therefore old source hashes will not match current files. Strict historical re-audit tools intentionally reject an unrelated source version. Use the archived source for exact historical reconstruction; use the organized source for new experiments. Never overwrite an old summary to pretend it was measured under the new layout.

Start learning with [the current project map](learning/01-project-map.md), then [the walking code walkthrough](learning/02-walking-code.md) and [the debugging guide](learning/03-debugging.md).
