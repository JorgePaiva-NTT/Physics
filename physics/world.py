
from .core import Vec2
from .constraints import TwoPointConstraint


class World:
    def __init__(self, width, height, gravity=Vec2(0, 981), friction=0.99):
        self.width = width
        self.height = height
        self.gravity = gravity
        self.friction = friction
        self.particles = []
        self.constraints = []

    def add_particle(self, particle):
        self.particles.append(particle)

    def add_constraint(self, constraint):
        self.constraints.append(constraint)

    def remove_particle(self, particle):
        if particle in self.particles:
            self.particles.remove(particle)
            self.constraints = [c for c in self.constraints if isinstance(c, TwoPointConstraint) and c.p1 != particle and c.p2 != particle]

    def update(self, dt):
        for p in self.particles:
            p.apply_force(self.gravity)
            p.update(dt)
            # Apply friction
            velocity = p.pos - p.old_pos
            p.old_pos = p.pos - velocity * (1.0 - self.friction)

        for _ in range(3): # Multiple iterations for stability
            for c in self.constraints:
                c.update()
            for p in self.particles:
                self._apply_boundary_constraints(p)


    def _apply_boundary_constraints(self, particle):
        # Wall constraints
        if particle.pos.x < particle.radius:
            particle.pos.x = particle.radius
        elif particle.pos.x > self.width - particle.radius:
            particle.pos.x = self.width - particle.radius
        if particle.pos.y < particle.radius:
            particle.pos.y = particle.radius
        elif particle.pos.y > self.height - particle.radius:
            particle.pos.y = self.height - particle.radius
