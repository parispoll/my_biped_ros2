# Command-line tools

Use the GUI for normal experiments. For CLI tools, source ROS/workspace and run from the workspace root:

`python3 -m scripts.<folder>.<tool> --help`

Each subfolder has a file map. `experiments/` runs individual simulations, `planning/` computes offline previews, `checks/` inspects measurements, `plots/` renders results, `analysis/` reads saved data, and `studies/` holds batch research comparisons.

`force_panel.py` and the two start shell scripts deliberately remain here as stable service entry points. Their location is part of the already-installed user service contract.
