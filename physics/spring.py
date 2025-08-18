
from .core import Vec2

class Spring:
    def __init__(self, p1, p2, length, stiffness):
        self.p1 = p1
        self.p2 = p2
        self.length = length
        self.stiffness = stiffness

    def update(self):
        direction = self.p2.pos - self.p1.pos
        distance = direction.length()
        displacement = distance - self.length
        force_magnitude = displacement * self.stiffness
        force = direction.normalize() * force_magnitude
        self.p1.apply_force(force)
        self.p2.apply_force(-force)

    def set_particles(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
