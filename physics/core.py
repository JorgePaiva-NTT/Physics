import math

class Vec2:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __truediv__(self, scalar):
        return Vec2(self.x / scalar, self.y / scalar)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def length(self):
        return math.hypot(self.x, self.y)

    def length_sq(self):
        return self.x * self.x + self.y * self.y

    def normalize(self):
        l = self.length()
        if l > 0.0:
            return Vec2(self.x / l, self.y / l)
        return Vec2(0.0, 0.0)

    def copy(self):
        return Vec2(self.x, self.y)

    def __eq__(self, other):
        if other is None or not isinstance(other, Vec2):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


class Particle:
    def __init__(self, pos, vel=None, radius=6, mass=1.0, fixed=False):
        self.pos = pos.copy() if isinstance(pos, Vec2) else Vec2(pos[0], pos[1])
        self.vel = vel.copy() if isinstance(vel, Vec2) else (vel if vel is not None else Vec2(0.0, 0.0))
        self.radius = radius
        self.mass = float(mass)
        self.fixed = fixed

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
