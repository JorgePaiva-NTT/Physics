from physics.Vec2 import Vec2


class Particle:
    def __init__(self, pos, vel=None, radius=6, mass=1.0, fixed=False):
        self.pos = pos.copy() if isinstance(pos, Vec2) else Vec2(pos[0], pos[1])
        self.vel = vel.copy() if isinstance(vel, Vec2) else (vel if vel is not None else Vec2(0.0, 0.0))
        self.radius = radius

        # Internal state for properties
        self._mass = 1.0
        self._fixed = False
        self.inv_mass = 1.0

        # Use setters to initialize and compute inv_mass
        self.mass = mass
        self.fixed = fixed

    @property
    def mass(self):
        return self._mass

    @mass.setter
    def mass(self, value):
        self._mass = float(value)
        self._update_inv_mass()

    @property
    def fixed(self):
        return self._fixed

    @fixed.setter
    def fixed(self, value):
        self._fixed = bool(value)
        self._update_inv_mass()

    def _update_inv_mass(self):
        if self._fixed or self._mass == 0:
            self.inv_mass = 0.0
        else:
            self.inv_mass = 1.0 / self._mass

    def update(self, dt):
        # Not used by RK4 integrator; kept for compatibility with older code paths.
        if self.fixed:
            return
        a = Vec2(0.0, 0.0)
        self.vel = Vec2(self.vel.x + a.x * dt, self.vel.y + a.y * dt)
        self.pos = Vec2(self.pos.x + self.vel.x * dt, self.pos.y + self.vel.y * dt)

    def apply_force(self, force):
        # compatibility no-op for RK4 world; other code may use it.
        pass

    def __eq__(self, other):
        if other is None or not isinstance(other, Particle):
            return False
        return self.pos == other.pos and self.radius == other.radius and self.mass == other.mass and self.fixed == other.fixed

    def __hash__(self):
        return hash((self.pos, self.radius, self.mass, self.fixed))

    def __repr__(self):
        return f"Particle(pos=({self.pos.x:.2f}, {self.pos.y:.2f}), vel=({self.vel.x:.2f}, {self.vel.y:.2f}), radius={self.radius}, mass={self.mass}, fixed={self.fixed})"

    def __str__(self):
        return self.__repr__()

    def to_dict(self):
        return {
            'pos': (self.pos.x, self.pos.y),
            'vel': (self.vel.x, self.vel.y),
            'radius': self.radius,
            'mass': self.mass,
            'fixed': self.fixed
        }