
import math

class Vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vec2(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        return Vec2(self.x / scalar, self.y / scalar)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self):
        l = self.length()
        if l > 0:
            return Vec2(self.x / l, self.y / l)
        return Vec2(0, 0)

    def __eq__(self, other):
        if (other == None): 
            return False
        if (not isinstance(other, Vec2)):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

class Particle:
    def __init__(self, pos, radius=10, mass=1.0, fixed=False):
        self.pos = pos
        self.old_pos = pos # Keep old_pos for Verlet integration
        self.acceleration = Vec2(0, 0) # Keep acceleration for force application
        self.radius = radius
        self.mass = mass
        self.fixed = fixed

    def update(self, dt):
        if self.fixed:
            return
        velocity = self.pos - self.old_pos
        self.old_pos = self.pos
        self.pos = self.pos + velocity + self.acceleration * dt * dt
        self.acceleration = Vec2(0, 0)

    def apply_force(self, force):
        if self.fixed:
            return
        self.acceleration += force / self.mass

    def __eq__(self, other):
        if (other == None): 
            return False
        if (not isinstance(other, Particle)):
            return False
        return self.pos == other.pos and self.radius == other.radius and self.mass == other.mass and self.fixed == other.fixed

    def __hash__(self):
        return hash((self.pos, self.radius, self.mass, self.fixed))
