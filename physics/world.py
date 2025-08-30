import math
try:
    import numpy as np
    _HAVE_NUMPY = True
except Exception:
    _HAVE_NUMPY = False

import pygame
from physics.solvers.rigidbody import RigidBody
from physics.spring import Spring
from physics.solvers.cloth import Cloth

import constants
from .Particle import Particle
from .Vec2 import Vec2
from .TwoPointConstraint import TwoPointConstraint
from .collision import detect_particle_collision, resolve_particle_collision


class World:
    def __init__(self, width, height, gravity=Vec2(0, 981), friction=0.1, restitution=0.5, cell_size=None, constraint_iterations=2):
        self.width = width
        self.height = height
        self.gravity = gravity
        self.friction = friction
        self.restitution = restitution  # Coefficient of restitution for collisions
        
        self.particles = []
        self.constraints = []
        self.groups = []
        self._next_group_id = 1  
        self._group_map = {} 
        
        # single pass collision resolution avoids back-and-forth corrections that cause flicker
        # increase this to 3-6 for stable stacking (tunable)
        # default tuned for stacked stability without too many passes
        self.collision_iterations = 4
        self.constraint_iterations = constraint_iterations

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

    def create_group(self, particles, stiffness=1.0, fixed=False):
        """
        Create a rigid group from an iterable of Particle objects or indices.
        Returns a group id.
        """
        # convert inputs to indices
        indices = []
        for p in particles:
            if isinstance(p, int):
                idx = p
            else:
                try:
                    idx = self.particles.index(p)
                except ValueError:
                    continue
            indices.append(idx)
        if not indices:
            return None

        # compute rest center and rest_rel
        rest_com_x = 0.0; rest_com_y = 0.0
        for i in indices:
            rest_com_x += self.particles[i].pos.x
            rest_com_y += self.particles[i].pos.y
        n = len(indices)
        rest_com_x /= n; rest_com_y /= n
        rest_rel = []
        for i in indices:
            p = self.particles[i]
            rest_rel.append(Vec2(p.pos.x - rest_com_x, p.pos.y - rest_com_y))

        group = RigidBody(indices, rest_rel, stiffness=stiffness, fixed=fixed)
        gid = self._next_group_id
        self._next_group_id += 1
        self._group_map[gid] = len(self.groups)
        self.groups.append(group)
        return gid

    def create_cloth(self, top_left_pos, width, height, segments_x, segments_y, tear_factor=3.0, pin_corners=None, stiffness=1.0):
        """
        Creates a cloth simulation object.

        :param top_left_pos: Vec2 position of the top-left corner.
        :param width: Total width of the cloth in pixels.
        :param height: Total height of the cloth in pixels.
        :param segments_x: Number of horizontal segments.
        :param segments_y: Number of vertical segments.
        :param tear_factor: How much a link can stretch before breaking. 0 to disable.
        :param pin_corners: A list/tuple of corners to pin, e.g., ['top_left', 'top_right'].
        :param stiffness: The stiffness of the distance constraints (0-1).
        :return: The group_id of the cloth solver.
        """
        
        num_particles_x = segments_x + 1
        num_particles_y = segments_y + 1
        dx = width / segments_x
        dy = height / segments_y

        particles = []
        for y in range(num_particles_y):
            for x in range(num_particles_x):
                pos = Vec2(top_left_pos.x + x * dx, top_left_pos.y + y * dy)
                p = Particle(pos, radius=4)
                particles.append(p)

        # Pinning logic
        if pin_corners:
            if 'top_left' in pin_corners: particles[0].fixed = True
            if 'top_right' in pin_corners: particles[num_particles_x - 1].fixed = True
            if 'bottom_left' in pin_corners: particles[num_particles_x * (num_particles_y - 1)].fixed = True
            if 'bottom_right' in pin_corners: particles[-1].fixed = True

        # Add particles to the world and get their indices
        start_index = len(self.particles)
        self.particles.extend(particles)
        indices = list(range(start_index, start_index + len(particles)))

        # Create, build, and add the cloth solver.
        cloth_solver = Cloth(indices, num_particles_x, num_particles_y, tear_factor, stiffness)
        cloth_solver.build_constraints(self)
        self.groups.append(cloth_solver)
        return cloth_solver # Returning the object itself can be useful

    def remove_group(self, group_id):
        idx = self._group_map.get(group_id)
        if idx is None:
            return
        self.groups.pop(idx)
        # rebuild map
        self._group_map = {gid: i for i, gid in enumerate(self._group_map.keys())}

    def add_particle_to_group(self, group_id, particle):
        idx = self._group_map.get(group_id)
        if idx is None:
            return False
        group = self.groups[idx]
        try:
            pi = self.particles.index(particle)
        except ValueError:
            return False
        if pi in group.indices:
            return True
        # append rest_rel using current com as reference (simple)
        # recompute rest_rel naively: keep existing rest_rel unchanged (user may rebuild)
        group.indices.append(pi)
        group.rest_rel.append(Vec2(0.0, 0.0))
        return True

    def set_group_fixed(self, group_id, fixed=True):
        idx = self._group_map.get(group_id)
        if idx is None:
            return
        group = self.groups[idx]
        group.fixed = bool(fixed)
        for i in group.indices:
            try:
                self.particles[i].fixed = bool(fixed)
            except Exception:
                pass

    def _solve_groups(self, dt):
        # Apply each group solver
        for g in self.groups:
            try:
                g.solve(self, dt)
            except Exception:
                pass

    def update(self, dt):
        """
        Updates the world state using a simple and stable Symplectic Euler integrator,
        followed by position-based constraint and collision resolution. This is a simpler
        and often faster alternative to RK4 for game physics.
        """
        n = len(self.particles)
        if n == 0:
            return

        # Store positions before integration for velocity update later (PBD style)
        pos_before_integration = [p.pos.copy() for p in self.particles]

        # --- Integration Step (Symplectic Euler) ---

        # 1. Compute all forces at the current state.
        forces = [Vec2(0.0, 0.0) for _ in range(n)]

        # Apply global forces (gravity, drag)
        for i, p in enumerate(self.particles):
            if not p.fixed:
                # Gravity
                forces[i] += self.gravity * p.mass
                # Viscous Drag (proportional to velocity)
                forces[i] -= p.vel * self.friction

        # Apply constraint forces (Springs)
        # Create an index map for efficient lookups
        index_map = {id(p): i for i, p in enumerate(self.particles)}
        for c in self.constraints:
            if isinstance(c, Spring):
                i = index_map.get(id(c.p1))
                j = index_map.get(id(c.p2))
                if i is None or j is None:
                    continue

                f_on_j = c.compute_force(c.p1.pos, c.p2.pos, c.p1.vel, c.p2.vel)
                forces[j] += f_on_j
                forces[i] -= f_on_j

        # 2. Update velocities using current accelerations
        for i, p in enumerate(self.particles):
            if not p.fixed and p.mass != 0:
                acceleration = forces[i] / p.mass
                p.vel += acceleration * dt

        # 3. Update positions using new velocities
        for p in self.particles:
            if not p.fixed:
                p.pos += p.vel * dt

        # --- Positional Correction & Collision Resolution ---

        # apply positional/constraint corrections (non-force constraints) over several iterations for stability
        for _ in range(max(1, int(self.constraint_iterations))):
            c: TwoPointConstraint
            for c in self.constraints:
                if not isinstance(c, Spring):  # Springs are force-based, already handled
                    try:
                        c.update(dt)
                    except Exception:
                        pass

            # Solve groups (like cloth, rigid bodies)
            for g in self.groups:
                try:
                    g.solve(self, dt)
                except Exception:
                    pass

        # apply boundary constraints once before collision resolution to keep constraints in-bounds
        for p in self.particles:
            self._apply_boundary_constraints(p)

        # Handle collisions
        self._handle_collisions()

        # Final boundary check after collisions
        for p in self.particles:
            self._apply_boundary_constraints(p)

        # --- Velocity Update (PBD style) ---
        # Recalculate velocities based on the change from the pre-constraint position.
        # This is crucial for stability as it syncs velocity with position corrections.
        # This step effectively replaces the velocity from the force integration step
        # with one that reflects the outcome of the constraint solves.
        for i, p in enumerate(self.particles):
            if not p.fixed and dt > 1e-9:
                p.vel = (p.pos - pos_before_integration[i]) / dt

        # --- Boundary Restitution (Post-Velocity-Update) ---
        # After the PBD velocity update, apply restitution for particles at boundaries.
        for p in self.particles:
            self._apply_boundary_restitution(p)

    def _apply_boundary_constraints(self, particle):
        if particle.fixed:
            return

        # This is a positional correction. It clamps the particle's position
        # to be within the world boundaries. The velocity-based bounce is
        # handled separately in _apply_boundary_restitution.
        if particle.pos.x < particle.radius:
            particle.pos.x = particle.radius
        elif particle.pos.x > self.width - particle.radius:
            particle.pos.x = self.width - particle.radius

        if particle.pos.y < particle.radius:
            particle.pos.y = particle.radius
        elif particle.pos.y > self.height - particle.radius:
            particle.pos.y = self.height - particle.radius

    def _apply_boundary_restitution(self, particle):
        """Applies velocity-based restitution for particles at boundaries."""
        if particle.fixed:
            return

        # Horizontal walls
        if (particle.pos.x <= particle.radius and particle.vel.x < 0) or \
           (particle.pos.x >= self.width - particle.radius and particle.vel.x > 0):
            particle.vel.x *= -self.bounce

        # Vertical walls
        if (particle.pos.y <= particle.radius and particle.vel.y < 0) or \
           (particle.pos.y >= self.height - particle.radius and particle.vel.y > 0):
            particle.vel.y *= -self.bounce
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
                                    
    def draw_particles(self, screen, selected_particle=None):
        """Helper to draw all particles in the world."""
        for p in self.particles:
            color = constants.RED
            if p.fixed:
                color = (125, 125, 125)
            elif p == selected_particle if selected_particle is not None else False:
                color = constants.BLUE
            pygame.draw.circle(screen, color, (int(p.pos.x), int(p.pos.y)), p.radius)
            if p.fixed:
                pygame.draw.circle(screen, constants.BLACK, (int(p.pos.x), int(p.pos.y)), p.radius*0.5)
                
    def draw_constraints(self, screen):
        """Helper to draw all constraints in the world."""
        for c in self.constraints:
            try:
                if isinstance(c, Spring):
                    c.draw(screen, color=(0, 150, 0), secondary_color=(0, 255, 0))
                else:
                    c.draw(screen, color=(0, 0, 0))
            except Exception:
                pass
        for g in self.groups:
            try:
                g.draw(screen, color=(150, 0, 150))
            except Exception:
                pass