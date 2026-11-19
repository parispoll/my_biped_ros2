# Visual feedback

Draw COM, support geometry and commanded-force markers. Visual rendering must not discard or replace measured evidence.

| File | Responsibility |
|---|---|
| [com_markers.py](com_markers.py) | Draw actual COM, its vertical projection, sole outlines and a short trail. |
| [com_visual.py](com_visual.py) | Optional GUI mailbox: marker-service delays never block the force loop. |
| [force_visual.py](force_visual.py) | Optional Gazebo GUI markers for a commanded sagittal force. |
