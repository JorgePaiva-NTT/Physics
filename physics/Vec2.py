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