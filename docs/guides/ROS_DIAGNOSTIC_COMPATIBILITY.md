# ROS Jazzy diagnostic library compatibility

On 17 November 2026, Gazebo failed before controller startup with an undefined `diagnostic_updater::Updater` constructor. The installed `controller_manager` was 4.44.0 (April build), while `diagnostic_updater` was 4.2.7 (November build). ROS diagnostics [issue 663](https://github.com/ros/diagnostics/issues/663) documents the constructor and object-layout ABI change between 4.2.6 and 4.2.7.

Administrator access was unavailable. To keep the project runnable without changing system packages, the official unmodified diagnostics 4.2.6 source was built under `.runtime/diagnostics-4.2.6/`. `.runtime/COLCON_IGNORE` keeps this dependency outside normal workspace package discovery.

`runtime/diagnostic_compat.py` checks the constructor symbols required by the installed controller against the system diagnostic library. It uses the isolated library only when the old required constructor is absent from the system library and present in the isolated build. `runtime/simulation.py` applies its library path only to the Gazebo child process. It does not replace files in `/opt/ros` or modify the user's global environment. With already compatible system packages, it does nothing.

Rebuild the isolated dependency if needed:

```bash
bash scripts/services/build_diagnostic_compat.sh
```

The official source is https://github.com/ros/diagnostics, tag `4.2.6`. This workaround restores the full old library/object layout; it is not a fabricated symbol alias between incompatible class layouts. A direct load check confirmed that the installed controller library loads with this isolated dependency.

A longer-term system repair is to align the relevant ROS control binaries and dependencies with one compatible repository snapshot. Review that package plan before using administrator access; a mixed partial update caused this problem.
