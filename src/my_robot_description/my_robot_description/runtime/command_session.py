"""Command-driven plan clock, independent of ROS and motor control.

Pausing is permitted only after a complete landing/reload/hold. A stop request
never freezes an airborne leg. The immutable geometric plan retains world feet.
"""


class CommandSession:
    def __init__(self, program):
        self.boundaries = [s["end"] for s in program.steps]
        self.elapsed = 0.0
        self.target = 0
        self.completed = 0
        self.last_stamp = None
        self.running = False

    def update(self, stamp, ready):
        previous, self.last_stamp = self.last_stamp, stamp
        if ready and self.running and previous is not None:
            # Sample just inside the completed hold, before the next support swap.
            limit = self.boundaries[self.target - 1] - 1e-6
            self.elapsed = min(limit, self.elapsed + max(0.0, stamp - previous))
            self.completed = sum(self.elapsed >= b - 2e-6 for b in self.boundaries)
            if self.elapsed >= limit:
                self.running = False
        return self.elapsed

    def command(self, action, steps, ready):
        if action == "status":
            return "Status requested"
        if not ready:
            raise ValueError("Robot is settling; wait for ready=true")
        if action in ("stop", "stand"):
            if self.running:
                # Finish whichever placement is in progress, then hold both feet.
                self.target = self.completed + 1
                return "Stopping after current placement and double-support hold"
            return "Already holding; stand preserves current foot positions"
        if action not in ("step", "walk"):
            raise ValueError("Use step, walk, stop, stand or status")
        if self.running:
            raise ValueError("Motion already active; use stop before another walk")
        count = 1 if action == "step" else steps
        if type(count) is not int or not 1 <= count <= len(self.boundaries):
            raise ValueError("steps must be an integer within the session capacity")
        if self.completed + count > len(self.boundaries):
            raise ValueError("Session capacity exceeded; relaunch for a fresh stance")
        self.target = self.completed + count
        self.running = True
        return f"Accepted {count} placement(s); target total {self.target}"

    def status(self, ready):
        return dict(
            ready=ready,
            state="settling" if not ready else "walking" if self.running else "holding",
            completed_steps=self.completed,
            target_steps=self.target,
            capacity_steps=len(self.boundaries),
            plan_time_s=self.elapsed,
        )
