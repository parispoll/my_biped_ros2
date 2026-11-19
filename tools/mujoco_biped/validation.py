"""Measure actual foot flight and landing, independently of the reference clock."""
import numpy as np


class PlacementObserver:
    """A placement needs >5 mm sole clearance, then 50 ms loaded on the floor.

    This is a basic migration check, not a stability proof or hardware validation.
    """
    def __init__(self, model, data):
        self.airborne = {s: False for s in ('left','right')}
        self.loaded_ticks = dict.fromkeys(self.airborne, 0)
        self.last_x = {s: float(data.body(s+'_foot').xpos[0]) for s in self.airborne}
        self.events = []
        self.model = model

    def update(self, data, forces):
        geometry = {}
        for side in self.airborne:
            geom = self.model.geom(side+'_foot')
            gid = geom.id
            # The eight actual box corners include sole tilt, not just center Z.
            corners = np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)]) * geom.size
            world = corners @ data.geom_xmat[gid].reshape(3,3).T + data.geom_xpos[gid]
            sole = float(min(world[:,2]))
            geometry[side] = dict(min_corner_height_m=sole,position_m=data.body(side+'_foot').xpos.tolist())
            if sole > .005 and forces[side] < 1:
                self.airborne[side] = True
                self.loaded_ticks[side] = 0
            if self.airborne[side]:
                self.loaded_ticks[side] = self.loaded_ticks[side]+1 if forces[side] > 5 and sole < .001 else 0
                if self.loaded_ticks[side] >= 5:
                    x = float(data.body(side+'_foot').xpos[0])
                    self.events.append(dict(side=side,time_s=float(data.time),advance_m=x-self.last_x[side]))
                    self.last_x[side] = x
                    self.airborne[side] = False
        return geometry
