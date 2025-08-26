from .core import Vec2
import math

class Spring:
    def __init__(self, p1, p2, length, stiffness):
        self.p1 = p1
        self.p2 = p2
        self.length = float(length)
        self.stiffness = float(stiffness)
        # If None, compute critical damping per-evaluation (recommended)
        self.damping = None

    def compute_force(self, p1_pos, p2_pos, p1_vel, p2_vel):
        """
        Return force on p2 (Vec2). Force on p1 is -force_on_p2.
        This is stateless and works with arbitrary positions/velocities
        (used by RK4 force evaluations).
        Robustness: guard against zero distance, NaNs, and clamp excessively large forces.
        """
        # vector from p1 to p2
        direction = Vec2(p2_pos.x - p1_pos.x, p2_pos.y - p1_pos.y)
        distance = direction.length()

        # guard: invalid distance or zero -> no force
        if not math.isfinite(distance) or distance == 0.0:
            return Vec2(0.0, 0.0)

        # normalized direction
        n = Vec2(direction.x / distance, direction.y / distance)

        # signed displacement (positive when stretched)
        x = distance - self.length

        # relative velocity along spring axis (signed)
        rel_vx = p2_vel.x - p1_vel.x
        rel_vy = p2_vel.y - p1_vel.y
        v_rel = rel_vx * n.x + rel_vy * n.y

        # effective mass (approx)
        try:
            m = (self.p1.mass + self.p2.mass) * 0.5
        except Exception:
            m = 1.0

        # critical damping if not provided
        critical_c = 2.0 * math.sqrt(max(self.stiffness * max(m, 1e-9), 0.0))
        c = self.damping if (self.damping is not None) else critical_c

        # spring + damper
        force_scalar = -(self.stiffness * x + c * v_rel)

        # guard against NaN / infinite force_scalar
        if not math.isfinite(force_scalar):
            return Vec2(0.0, 0.0)

        # clamp to prevent extreme impulses (tunable)
        max_force = max(1e4, abs(self.stiffness) * 1e3)
        if abs(force_scalar) > max_force:
            force_scalar = math.copysign(max_force, force_scalar)

        return Vec2(n.x * force_scalar, n.y * force_scalar)

    def update(self, dt=None):
        """
        Backwards-compatible update: evaluate using current particle states and
        apply forces via a compatibility path (no accumulator in RK4 world).
        Left for legacy paths.
        """
        try:
            f_on_p2 = self.compute_force(self.p1.pos, self.p2.pos, self.p1.vel, self.p2.vel)
        except Exception:
            return
        # legacy apply_force may be a no-op under RK4 core; call if present
        try:
            self.p1.apply_force(Vec2(-f_on_p2.x, -f_on_p2.y))
            self.p2.apply_force(f_on_p2)
        except Exception:
            pass

    def set_particles(self, p1, p2):
        self.p1 = p1
        self.p2 = p2