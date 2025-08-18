
from .core import Vec2

class TwoPointConstraint:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def set_particles(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        
    def set_particles(self, p1, p2):
        self.p1 = p1
        self.p2 = p2


class DistanceConstraint(TwoPointConstraint):
    def __init__(self, p1, p2, distance):
        TwoPointConstraint.__init__(self, p1, p2)
        self.distance = distance

    def update(self):
        axis = self.p1.pos - self.p2.pos
        dist = axis.length()
        if dist == 0:
            return
        n = axis / dist
        delta = self.distance - dist
        self.p1.pos += n * delta * 0.5
        self.p2.pos -= n * delta * 0.5

class PinConstraint:
    def __init__(self, p, pos):
        self.p = p
        self.pos = pos

    def update(self):
        self.p.pos = self.pos

    def set_particle(self, p):
        self.p = p

class FloorConstraint:
    def __init__(self, p, y):
        self.p = p
        self.y = y

    def update(self):
        if self.p.pos.y > self.y:
            self.p.pos.y = self.y

    def set_particle(self, p):
        self.p = p
    

