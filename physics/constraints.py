from .core import Vec2

class TwoPointConstraint:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def set_particles(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

class DistanceConstraint(TwoPointConstraint):
    def __init__(self, p1, p2, distance, percent=0.05, slop=None):
        super().__init__(p1, p2)
        self.distance = float(distance)
        # positional correction parameters tuned for cloth stability:
        # - percent: reduced correction strength (5%)
        # - slop: small allowed tolerance to avoid jitter; default = max(0.01, 5% of rest length)
        self.percent = float(percent)
        self.slop = slop if (slop is not None) else max(0.01, self.distance * 0.05)

    def update(self):
        # positional (non-force) correction to satisfy target distance
        p1 = self.p1
        p2 = self.p2
        delta = p2.pos - p1.pos
        dist = delta.length()
        if dist == 0.0:
            return

        # difference (positive when stretched)
        diff = dist - self.distance
        if abs(diff) <= self.slop:
            return

        # compute mass-weighted correction (respect fixed flags)
        inv_m1 = 0.0 if getattr(p1, "fixed", False) else (1.0 / p1.mass)
        inv_m2 = 0.0 if getattr(p2, "fixed", False) else (1.0 / p2.mass)
        inv_sum = inv_m1 + inv_m2
        if inv_sum == 0.0:
            return

        # correction vector: normalized(delta) * diff * percent
        correction_mag = (diff / dist) * self.percent
        correction = Vec2(delta.x * correction_mag, delta.y * correction_mag)

        # apply mass-weighted positional corrections only (no velocity zeroing)
        if not getattr(p1, "fixed", False):
            factor1 = inv_m1 / inv_sum
            p1.pos = Vec2(p1.pos.x - correction.x * factor1, p1.pos.y - correction.y * factor1)

        if not getattr(p2, "fixed", False):
            factor2 = inv_m2 / inv_sum
            p2.pos = Vec2(p2.pos.x + correction.x * factor2, p2.pos.y + correction.y * factor2)


class PinConstraint:
    def __init__(self, p, pos):
        self.p = p
        # store a Vec2 copy of pinned location
        self.pos = pos.copy() if hasattr(pos, "copy") else Vec2(pos.x, pos.y)
        # mark particle fixed for RK4 integrator so solver treats it immovable
        try:
            self.p.fixed = True
        except Exception:
            pass

    def update(self):
        # enforce pinned position each frame (positional correction)
        try:
            self.p.pos = self.pos.copy()
            if hasattr(self.p, "vel"):
                self.p.vel = Vec2(0.0, 0.0)
        except Exception:
            pass

    def set_particle(self, p):
        # un-fix previous particle and fix new one
        try:
            if hasattr(self, "p") and self.p is not None:
                self.p.fixed = False
        except Exception:
            pass
        self.p = p
        try:
            self.p.fixed = True
        except Exception:
            pass


