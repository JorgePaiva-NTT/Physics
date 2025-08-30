from .Vec2 import Vec2

def detect_particle_collision(p1, p2):
    """
    Detect collision between two particles.
    Returns True if particles are colliding, False otherwise.
    """
    # use squared distance to avoid unnecessary sqrt for simple test
    dx = p1.pos.x - p2.pos.x
    dy = p1.pos.y - p2.pos.y
    dist_sq = dx * dx + dy * dy
    rsum = (p1.radius + p2.radius)
    return dist_sq < (rsum * rsum)

def resolve_particle_collision(p1, p2, 
                               restitution=0.3, 
                               percent=1, 
                               slop=0.001, 
                               max_correction=0.9):
    """
    Resolve collision between two particles using impulse-based collision response
    with improved positional correction to avoid deep penetration and flicker.

    - restitution: bounciness
    - percent: positional correction factor (0..1). Higher fixes penetration faster.
    - slop: small penetration allowed before correction.
    - max_correction: clamp positional correction magnitude to avoid large jumps.
    """
    if p1.fixed and p2.fixed:
        return

    # vector from p1 to p2
    delta = p2.pos - p1.pos
    dist = delta.length()
    # handle coincident centers
    if dist <= 1e-9:
        # push along an arbitrary axis to avoid NaNs
        normal = Vec2(1.0, 0.0)
        dist = 1e-9
    else:
        normal = Vec2(delta.x / dist, delta.y / dist)

    # relative velocity along normal
    rv = Vec2(p2.vel.x - p1.vel.x, p2.vel.y - p1.vel.y)
    vel_along_normal = rv.x * normal.x + rv.y * normal.y

    # inverse masses
    inv_mass1 = 0.0 if p1.fixed else (1.0 / p1.mass if p1.mass != 0 else 0.0)
    inv_mass2 = 0.0 if p2.fixed else (1.0 / p2.mass if p2.mass != 0 else 0.0)
    inv_mass_sum = inv_mass1 + inv_mass2
    if inv_mass_sum == 0.0:
        return

    # penetration depth
    penetration = (p1.radius + p2.radius) - dist

    # If there is significant penetration, perform positional correction first.
    if penetration > slop:
        # compute correction magnitude (distribute by inverse-mass)
        correction_mag = (penetration - slop) * percent
        # clamp to avoid huge jumps for very deep overlaps
        correction_mag = min(correction_mag, max_correction)
        # distribute correction proportionally to inverse mass
        if not p1.fixed:
            share1 = inv_mass1 / inv_mass_sum
            p1.pos = Vec2(p1.pos.x - normal.x * correction_mag * share1,
                          p1.pos.y - normal.y * correction_mag * share1)
        if not p2.fixed:
            share2 = inv_mass2 / inv_mass_sum
            p2.pos = Vec2(p2.pos.x + normal.x * correction_mag * share2,
                          p2.pos.y + normal.y * correction_mag * share2)

    # Only apply impulse if bodies are approaching (prevents adding energy in separating contacts)
    if vel_along_normal < 0.0:
        # In a PBD-style solver, velocity updates are derived from position changes.
        # The impulse-based velocity change is removed to prevent conflicting updates
        # and instability. The positional correction above is now solely responsible
        # for resolving the collision. Bounciness (restitution) is implicitly handled
        # by the main world's velocity update, though less directly than with impulses.
        pass
