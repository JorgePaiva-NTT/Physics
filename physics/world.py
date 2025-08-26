import math
try:
    import numpy as np
    _HAVE_NUMPY = True
except Exception:
    _HAVE_NUMPY = False

from physics.spring import Spring
from .core import Vec2
from .constraints import TwoPointConstraint
from .collision import detect_particle_collision, resolve_particle_collision


class World:
    def __init__(self, width, height, gravity=Vec2(0, 981), friction=0.01, restitution=0.5, cell_size=None):
        self.width = width
        self.height = height
        self.gravity = gravity
        self.friction = friction
        self.restitution = restitution  # Coefficient of restitution for collisions
        self.particles = []
        self.constraints = []
        # single pass collision resolution avoids back-and-forth corrections that cause flicker
        # increase this to 3-6 for stable stacking (tunable)
        # default tuned for stacked stability without too many passes
        self.collision_iterations = 4

        self.bounce = restitution  # use restitution for wall collisions
        
        if cell_size is None:
            self.cell_size = max(8.0, min(self.width, self.height) / 10.0)
        else: 
            self.cell_size = float(cell_size)
        self._spatial_hash = {}

        # Verlet neighbor list (verlet list) to reduce pair checks.
        # Rebuild every `verlet_rebuild_freq` frames (K). Small `verlet_skin` adds margin.
        # default rebuild frequency lowered to keep lists fresh when many particles move
        self.verlet_rebuild_freq = 5        # K frames; tune (e.g. 3-10)
        # smaller skin reduces unnecessary neighbors while keeping a safety margin
        self.verlet_skin = max(0.5, self.cell_size * 0.4)
        self._verlet_list = None            # list of neighbor index lists per particle
        self._verlet_frame_counter = 0
        self._verlet_last_n = 0

    def add_particle(self, particle):
        self.particles.append(particle)

    def add_constraint(self, constraint):
        self.constraints.append(constraint)

    def remove_particle(self, particle):
        if particle in self.particles:
            self.particles.remove(particle) 
            # remove any constraints involving this particle
            self.constraints = [c for c in self.constraints if not (hasattr(c, 'p1') and c.p1 == particle) and not (hasattr(c, 'p2') and c.p2 == particle)]

    def update(self, dt):
        """
        Full-system RK4 integrator.
        - Forces: gravity, viscous drag (proportional to vel), and springs (force-based).
        - Positional constraints (PinConstraint, DistanceConstraint) are applied after integration
          as correction steps (they are assumed to be positional constraints).
        - Collisions and boundaries are resolved after integration.
        """
        n = len(self.particles)
        if n == 0:
            return

        # Create index map for quick lookup (use id(p) to avoid mutable-hash issues)
        index_map = {id(p): i for i, p in enumerate(self.particles)}

        # initial state arrays
        pos0 = [p.pos.copy() for p in self.particles]
        vel0 = [p.vel.copy() for p in self.particles]
        masses = [p.mass for p in self.particles]
        fixed = [p.fixed for p in self.particles]

        def compute_accelerations(positions, velocities):
            # returns list of Vec2 accelerations
            forces = [Vec2(0.0, 0.0) for _ in range(n)]

            # external forces: gravity and viscous damping
            for i in range(n):
                if fixed[i]:
                    continue
                # gravity
                forces[i] = Vec2(self.gravity.x * masses[i], self.gravity.y * masses[i])
                # viscous drag proportional to velocity
                forces[i] = Vec2(forces[i].x - velocities[i].x * self.friction * masses[i],
                                  forces[i].y - velocities[i].y * self.friction * masses[i])

            # springs (force-based)
            for c in self.constraints:
                if isinstance(c, Spring):
                    i = index_map.get(id(c.p1))
                    j = index_map.get(id(c.p2))
                    if i is None or j is None:
                        # constraint refers to particle not in current world list; skip
                        continue
                    f_on_j = c.compute_force(positions[i], positions[j], velocities[i], velocities[j])
                    # apply force: + on j, - on i
                    forces[j] = Vec2(forces[j].x + f_on_j.x, forces[j].y + f_on_j.y)
                    forces[i] = Vec2(forces[i].x - f_on_j.x, forces[i].y - f_on_j.y)

            # accelerations
            accs = []
            for i in range(n):
                if fixed[i]:
                    accs.append(Vec2(0.0, 0.0))
                else:
                    accs.append(Vec2(forces[i].x / masses[i], forces[i].y / masses[i]))
            return accs

        h = dt

        # k1
        a1 = compute_accelerations(pos0, vel0)
        dp1 = [v.copy() for v in vel0]
        dv1 = [a.copy() for a in a1]

        # k2
        pos_k2 = [Vec2(pos0[i].x + dp1[i].x * (h * 0.5), pos0[i].y + dp1[i].y * (h * 0.5)) for i in range(n)]
        vel_k2 = [Vec2(vel0[i].x + dv1[i].x * (h * 0.5), vel0[i].y + dv1[i].y * (h * 0.5)) for i in range(n)]
        a2 = compute_accelerations(pos_k2, vel_k2)
        dp2 = [v.copy() for v in vel_k2]
        dv2 = [a.copy() for a in a2]

        # k3
        pos_k3 = [Vec2(pos0[i].x + dp2[i].x * (h * 0.5), pos0[i].y + dp2[i].y * (h * 0.5)) for i in range(n)]
        vel_k3 = [Vec2(vel0[i].x + dv2[i].x * (h * 0.5), vel0[i].y + dv2[i].y * (h * 0.5)) for i in range(n)]
        a3 = compute_accelerations(pos_k3, vel_k3)
        dp3 = [v.copy() for v in vel_k3]
        dv3 = [a.copy() for a in a3]

        # k4
        pos_k4 = [Vec2(pos0[i].x + dp3[i].x * h, pos0[i].y + dp3[i].y * h) for i in range(n)]
        vel_k4 = [Vec2(vel0[i].x + dv3[i].x * h, vel0[i].y + dv3[i].y * h) for i in range(n)]
        a4 = compute_accelerations(pos_k4, vel_k4)
        dp4 = [v.copy() for v in vel_k4]
        dv4 = [a.copy() for a in a4]

        # combine to next state
        pos_next = []
        vel_next = []
        for i in range(n):
            if fixed[i]:
                pos_next.append(pos0[i].copy())
                vel_next.append(Vec2(0.0, 0.0))
                continue
            px = pos0[i].x + (h / 6.0) * (dp1[i].x + 2.0 * dp2[i].x + 2.0 * dp3[i].x + dp4[i].x)
            py = pos0[i].y + (h / 6.0) * (dp1[i].y + 2.0 * dp2[i].y + 2.0 * dp3[i].y + dp4[i].y)
            vx = vel0[i].x + (h / 6.0) * (dv1[i].x + 2.0 * dv2[i].x + 2.0 * dv3[i].x + dv4[i].x)
            vy = vel0[i].y + (h / 6.0) * (dv1[i].y + 2.0 * dv2[i].y + 2.0 * dv3[i].y + dv4[i].y)
            pos_next.append(Vec2(px, py))
            vel_next.append(Vec2(vx, vy))

        # commit new state
        for i, p in enumerate(self.particles):
            # safety: avoid committing NaN/Inf from integrator — revert to previous state if detected
            px = pos_next[i].x
            py = pos_next[i].y
            vx = vel_next[i].x
            vy = vel_next[i].y
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(vx) and math.isfinite(vy)):
                # skip update for this particle (keep previous pos/vel)
                continue
            p.pos = pos_next[i]
            p.vel = vel_next[i]

        # apply positional/constraint corrections (non-force constraints)
        for c in self.constraints:
            if isinstance(c, Spring):
                # springs are force-based and already applied in RK4 -> skip positional correction
                continue
            else:
                try:
                    c.update()
                except Exception:
                    pass

        # apply boundary constraints once before collision resolution to keep constraints in-bounds
        for p in self.particles:
            self._apply_boundary_constraints(p)

        # single-pass collision resolution (positional + impulse), then final clamp
        self._handle_collisions()
        for p in self.particles:
            self._apply_boundary_constraints(p)

    def _apply_boundary_constraints(self, particle):
        if particle.fixed:
            return

        if particle.pos.x < particle.radius:
            particle.pos.x = particle.radius
            if particle.vel.x < 0:
                particle.vel.x = -particle.vel.x * self.bounce
        elif particle.pos.x > self.width - particle.radius:
            particle.pos.x = self.width - particle.radius
            if particle.vel.x > 0:
                particle.vel.x = -particle.vel.x * self.bounce

        if particle.pos.y < particle.radius:
            particle.pos.y = particle.radius
            if particle.vel.y < 0:
                particle.vel.y = -particle.vel.y * self.bounce
        elif particle.pos.y > self.height - particle.radius:
            particle.pos.y = self.height - particle.radius
            if particle.vel.y > 0:
                particle.vel.y = -particle.vel.y * self.bounce

    def _rebuild_verlet_list(self, cutoff=None):
        """
        Build a Verlet neighbor list for current particles.
        cutoff: optional search radius; if None, computed as 2*max_radius + skin
        Uses the existing spatial-hash helper cell size for broad-phase.
        """
        n = len(self.particles)
        if n == 0:
            self._verlet_list = []
            self._verlet_last_n = 0
            return

        # compute cutoff if not provided
        max_r = max((p.radius for p in self.particles), default=0.0)
        if cutoff is None:
            cutoff = 2.0 * max_r + self.verlet_skin
        cutoff_sq = cutoff * cutoff

        cs = max(1e-6, self.cell_size)

        # build spatial hash (temporary local)
        spatial = {}
        def key_for(p):
            return (int(p.pos.x // cs), int(p.pos.y // cs))
        for idx, p in enumerate(self.particles):
            k = key_for(p)
            spatial.setdefault(k, []).append(idx)

        # neighbor search within neighboring cells
        neigh = [[] for _ in range(n)]
        for (cx, cy), idx_list in spatial.items():
            # check pairs inside this cell and neighbors
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    nk = (cx + dx, cy + dy)
                    if nk not in spatial:
                        continue
                    other_list = spatial[nk]
                    for i in idx_list:
                        for j in other_list:
                            if i >= j:
                                continue
                            p1 = self.particles[i]
                            p2 = self.particles[j]
                            dxp = p2.pos.x - p1.pos.x
                            dyp = p2.pos.y - p1.pos.y
                            if (dxp * dxp + dyp * dyp) <= cutoff_sq:
                                neigh[i].append(j)
                                neigh[j].append(i)
        self._verlet_list = neigh
        self._verlet_last_n = n
        self._verlet_frame_counter = 0

    def _handle_collisions(self):
        """
        Broad-phase via spatial hash + narrow-phase. Perform several resolution iterations
        to converge stacked contacts and reduce order-dependent flicker.

        Uses a Verlet neighbor list rebuilt every K frames (self.verlet_rebuild_freq)
        to avoid rebuilding the spatial hash / neighbor search every frame.
        """
        # allow user to tune how many iterations of collision resolution to run
        iterations = max(1, int(self.collision_iterations))

        for it in range(iterations):
            # decide whether to rebuild Verlet list
            n = len(self.particles)
            rebuild = False
            if self._verlet_list is None:
                rebuild = True
            elif self._verlet_last_n != n:
                rebuild = True
            elif (self._verlet_frame_counter % max(1, int(self.verlet_rebuild_freq))) == 0:
                rebuild = True

            if rebuild:
                # rebuild verlet list (uses spatial hashing internally)
                self._rebuild_verlet_list()
            else:
                self._verlet_frame_counter += 1

            # use verlet neighbor list if present
            if self._verlet_list is not None and len(self._verlet_list) == n:
                # Build list of unique unordered pairs once (Python loop but cheap relative to many collision ops)
                pairs = []
                for i, neigh in enumerate(self._verlet_list):
                    for j in neigh:
                        if i < j:
                            pairs.append((i, j))

                if not pairs:
                    # nothing to test this iteration
                    self._verlet_frame_counter += 1
                    continue

                # Vectorized distance prefilter with numpy when available
                if _HAVE_NUMPY and len(pairs) > 64:
                    arr = np.asarray(pairs, dtype=np.int32)
                    i_idxs = arr[:, 0]
                    j_idxs = arr[:, 1]

                    # gather position and radius arrays once
                    pos_x = np.fromiter((p.pos.x for p in self.particles), dtype=np.float64)
                    pos_y = np.fromiter((p.pos.y for p in self.particles), dtype=np.float64)
                    radii = np.fromiter((p.radius for p in self.particles), dtype=np.float64)

                    dx = pos_x[j_idxs] - pos_x[i_idxs]
                    dy = pos_y[j_idxs] - pos_y[i_idxs]
                    dist2 = dx * dx + dy * dy
                    rsum = radii[i_idxs] + radii[j_idxs]
                    rsum2 = rsum * rsum

                    mask = dist2 < rsum2
                    # iterate only actual colliding pairs and call narrow-phase / resolve
                    coll_indices = np.nonzero(mask)[0]
                    for k in coll_indices:
                        i = int(i_idxs[k]); j = int(j_idxs[k])
                        p1 = self.particles[i]
                        p2 = self.particles[j]
                        # double-check with existing narrow-phase and resolve
                        if detect_particle_collision(p1, p2):
                            resolve_particle_collision(p1, p2,
                                                       restitution=self.restitution,
                                                       percent=0.2, slop=0.01, max_correction=0.5)
                else:
                    # fallback: iterate pairs in Python (cheap for small lists or when numpy unavailable)
                    for i, j in pairs:
                        p1 = self.particles[i]
                        p2 = self.particles[j]
                        dxp = p2.pos.x - p1.pos.x
                        dyp = p2.pos.y - p1.pos.y
                        rsum = p1.radius + p2.radius
                        if (dxp * dxp + dyp * dyp) > (rsum * rsum):
                            continue
                        if detect_particle_collision(p1, p2):
                            resolve_particle_collision(p1, p2,
                                                       restitution=self.restitution,
                                                       percent=0.2, slop=0.01, max_correction=0.5)

                # increment frame counter (we used this iteration)
                self._verlet_frame_counter += 1
                continue

            # fallback: spatial-hash broad-phase like before (when verlet not available)
            self._spatial_hash.clear()
            cs = self.cell_size

            if cs <= 0:
                for i in range(len(self.particles)):
                    for j in range(i + 1, len(self.particles)):
                        p1 = self.particles[i]
                        p2 = self.particles[j]
                        if detect_particle_collision(p1, p2):
                            resolve_particle_collision(p1, p2,
                                                       restitution=self.restitution,
                                                       percent=0.2, slop=0.01, max_correction=0.5)
                continue

            def cell_key_for_pos(x, y):
                return (int(math.floor(x / cs)), int(math.floor(y / cs)))

            for idx, p in enumerate(self.particles):
                key = cell_key_for_pos(p.pos.x, p.pos.y)
                self._spatial_hash.setdefault(key, []).append(idx)

            checked = set()

            for (cx, cy), idx_list in list(self._spatial_hash.items()):
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        neighbor_key = (cx + dx, cy + dy)
                        if neighbor_key not in self._spatial_hash:
                            continue
                        neighbor_idx_list = self._spatial_hash[neighbor_key]
                        for i in idx_list:
                            for j in neighbor_idx_list:
                                if i >= j:
                                    continue
                                pair = (min(i, j), max(i, j))
                                if pair in checked:
                                    continue
                                checked.add(pair)
                                p1 = self.particles[i]
                                p2 = self.particles[j]

                                # optional AABB pre-check: radius-based early reject
                                dxp = p2.pos.x - p1.pos.x
                                dyp = p2.pos.y - p1.pos.y
                                rsum = p1.radius + p2.radius
                                if (dxp * dxp + dyp * dyp) > (rsum * rsum):
                                    continue
                                # narrow-phase test & resolution (use world restitution + gentler positional correction)
                                if detect_particle_collision(p1, p2):
                                    resolve_particle_collision(p1, p2,
                                                               restitution=self.restitution,
                                                               percent=0.2, slop=0.01, max_correction=0.5)

