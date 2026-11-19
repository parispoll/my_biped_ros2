"""Static plot of RoboKit kinematics; not measured Gazebo evidence."""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot(folder):
    rows = json.loads((folder / 'trajectory.json').read_text())
    t = [r['time_s'] for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
    for side, color in [('left', '#268078'), ('right', '#db8133')]:
        feet = np.array([r['feet_world_m'][side] for r in rows])
        axes[0].plot(t, 1000 * (feet[:,0] - feet[0,0]), label=f'{side} foot', color=color)
        axes[1].plot(t, 1000 * (feet[:,2] - feet[0,2]), label=f'{side} foot', color=color)
    axes[0].set_ylabel('Forward travel (mm)')
    axes[1].set_ylabel('Lift above initial pose (mm)')
    axes[1].set_xlabel('Original first-step clock (s)')
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend()
    fig.suptitle('75 mm biped step — offline RoboKit reproduction\nKinematics only; no physics execution', fontsize=14)
    fig.savefig(folder / 'step-reproduction.png', dpi=160)
    plt.close(fig)
