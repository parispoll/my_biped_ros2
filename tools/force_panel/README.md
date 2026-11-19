# Local experiment panel

The panel is a client of the experiment runners, not a motor controller.

- `static/index.html`: form and results structure.
- `static/app.js`: preview labels, submit/stop/re-audit actions, progress and history.
- `static/app.css`: presentation only.
- `config.py`: validate typed settings and construct a fixed allowed runner command.
- `jobs.py`: own one experiment process, prerequisites, cancellation, saved metadata and roadmap.
- `server.py`: authenticated loopback HTTP routes and static files.

The stable entry point remains `scripts/force_panel.py`, started by `scripts/start_force_panel.sh`. Scientific checks belong in the Python library's `validation/` folder. Keep visual changes separate from controller behavior.
