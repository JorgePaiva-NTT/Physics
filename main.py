import pygame
from pygame import mouse
from constants import BLACK, EMITTER_COLOR, EMITTER_SIZE, FPS, HEIGHT, WHITE, WIDTH
from physics.DistanceConstraint import DistanceConstraint
from physics.Particle import Particle
from physics.TwoPointConstraint import TwoPointConstraint
from physics.world import World
from physics.Vec2 import Vec2
from physics.spring import Spring
from physics.serialization import save_world, load_world
from physics.emitter import Emitter

from multiprocessing import Process, Manager
import gui_controller as gui_ctrl

# helper functions (keep existing implementations)
def get_particle_under_cursor(mx, my, world):
    for p in world.particles:
        dx = p.pos.x - mx
        dy = p.pos.y - my
        if (dx * dx + dy * dy) <= (p.radius + 3) ** 2:
            return p
    return None

def get_constraint_under_cursor(mx, my, world):
    for c in world.constraints:
        # Check distance from point to line segment
        p1, p2 = c.p1.pos, c.p2.pos
        line_mag = (p2 - p1).length()
        if line_mag < 0.000001:
            continue
        u = ((mx - p1.x) * (p2.x - p1.x) + (my - p1.y) * (p2.y - p1.y)) / (line_mag * line_mag)
        if u < 0 or u > 1:
            continue
        ix = p1.x + u * (p2.x - p1.x)
        iy = p1.y + u * (p2.y - p1.y)
        dist = ((ix - mx) ** 2 + (iy - my) ** 2) ** 0.5
        if dist < 5:  # threshold
            return c
    return None

def _move_particle(particle, new_pos):
    particle.pos = Vec2(new_pos.x, new_pos.y)
    particle.prev_pos = Vec2(new_pos.x, new_pos.y)  # Prevent snapping back

def _mouse_motion(event, particle=None):
    mx, my = event.pos
    if particle:
        particle.fixed = True
        particle.vel = Vec2(0, 0)
        _move_particle(particle, Vec2(mx, my))

def setup_simulation(world=None):
    if world is None:
        return None
    p1 = Particle(Vec2(300, 200), fixed=True)
    p2 = Particle(Vec2(400, 200))
    world.add_particle(p1)
    world.add_particle(p2)
    world.add_constraint(Spring(p1, p2, (p1.pos - p2.pos).length(), 15))  # Changed from 25

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Physics Playground")
    clock = pygame.time.Clock()

    # --- Physics World ---
    world = World(WIDTH, HEIGHT, gravity=Vec2(0, 981), friction=0.1, restitution=0.5, constraint_iterations=20)

    # --- Simulation Setup ---
    setup_simulation(world)

    emitter = None
    input_emitter_rate_mode = False
    input_emitter_rate_text = ''

    # --- Main Loop state ---
    running = True
    selected_particle = None
    first_particle_for_constraint = None
    first_particle_for_spring = None
    paused = False
    cloth_solver = None

    font = pygame.font.Font(None, 32)

    # spawn DearPyGui controller process (protected inside main)
    _mgr = Manager()
    _shared = _mgr.dict()
    _shared['verlet_rebuild_freq'] = world.verlet_rebuild_freq
    _shared['verlet_skin'] = world.verlet_skin
    _shared['collision_iterations'] = world.collision_iterations
    _shared['constraint_iterations'] = world.constraint_iterations
    _shared['reset_world'] = False
    _shared['spawn_emitter'] = False
    _shared['selected_constraint'] = None  # to be set by main loop when a constraint is selected
    _shared['__exit__'] = False
    _gui_proc = Process(target=gui_ctrl.run_gui, args=(_shared,), daemon=True)
    _gui_proc.start()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                selected_particle = get_particle_under_cursor(mx, my, world)
                selected_constraint = get_constraint_under_cursor(mx, my, world)
                if event.button == 1 and not selected_particle and not selected_constraint:
                    world.add_particle(Particle(Vec2(mx, my)))
                elif event.button == 1 and selected_constraint:
                    # store index so main process can find the exact constraint later
                    try:
                        idx = world.constraints.index(selected_constraint)
                    except ValueError:
                        idx = None
                    if idx is not None:
                        cur = _shared.get('selected_constraint')
                        # toggle selection if same constraint clicked twice
                        if cur and isinstance(cur, dict) and cur.get('index') == idx:
                            _shared['selected_constraint'] = None
                        else:
                            _shared['selected_constraint'] = {
                                'type': type(selected_constraint).__name__,
                                'index': idx,
                                'length': getattr(selected_constraint, 'length', getattr(selected_constraint, 'rest_length', None)),
                                'stiffness': getattr(selected_constraint, 'stiffness', getattr(selected_constraint, 'k', None)),
                                'damping': getattr(selected_constraint, 'damping', None)
                            }
                            # seed GUI fields
                            _shared['spring_length'] = getattr(selected_constraint, 'length', getattr(selected_constraint, 'rest_length', 100.0))
                            _shared['spring_stiffness'] = getattr(selected_constraint, 'stiffness', getattr(selected_constraint, 'k', 15.0))
                            _shared['spring_damping'] = getattr(selected_constraint, 'damping', 0.0)
                elif event.button == 1 and selected_constraint and _shared['selected_constraint'] is not None:
                    _shared['selected_constraint'] = None
                elif event.button == 3:
                    if emitter is None:
                        emitter = Emitter(Vec2(mx, my), direction=Vec2(0, -1), velocity=500, rate=0.05)
                    else:
                        emitter = None
            elif event.type == pygame.MOUSEBUTTONUP:
                if (selected_particle):
                    selected_particle.fixed = False
                    selected_particle = None
            elif event.type == pygame.MOUSEMOTION:
                _mouse_motion(event, selected_particle)
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    # Reset world
                    world = World(WIDTH, HEIGHT, gravity=Vec2(0, 981), friction=0.1, restitution=0.5, constraint_iterations=world.constraint_iterations)
                    emitter = None
                elif event.key == pygame.K_p:
                    # Save world
                    save_world(world, 'savefile.json')
                elif event.key == pygame.K_l:
                    # Load world
                    loaded_world = load_world('savefile.json')
                    if loaded_world:
                        world = loaded_world
                        emitter = None
                elif event.key == pygame.K_d:
                    mx, my = mouse.get_pos()
                    if first_particle_for_constraint:
                        other_particle = get_particle_under_cursor(mx, my, world)
                        world.add_constraint(DistanceConstraint(first_particle_for_constraint, other_particle, (first_particle_for_constraint.pos - other_particle.pos).length()))
                        first_particle_for_constraint = None
                    else:
                        first_particle_for_constraint = get_particle_under_cursor(mx, my, world)
                elif event.key == pygame.K_f:
                    # set particle fixed
                    if selected_particle:
                        selected_particle.fixed = True
                        selected_particle = None
                elif event.key == pygame.K_s:
                    # create spring between two particles
                    if selected_particle:
                        if first_particle_for_spring is None:
                            first_particle_for_spring = selected_particle
                            selected_particle = None
                        else:
                            if first_particle_for_spring != selected_particle:
                                rest_length = (first_particle_for_spring.pos - selected_particle.pos).length()
                                world.add_constraint(Spring(first_particle_for_spring, selected_particle, rest_length, 25))
                            first_particle_for_spring = None
                            selected_particle = None
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_c:
                    cloth_solver = world.create_cloth(
                        top_left_pos=Vec2(200, 50),
                        width=400,
                        height=300,
                        segments_x=25,
                        segments_y=20,
                        tear_factor=0,
                        pin_corners=['top_left', 'top_right'],
                        stiffness=1.0 # How much of the error to correct each iteration (0-1)
                    )
                    paused = not paused
                else:
                    pass

        # --- Handle GUI updates ---
        try:
            if _shared.get('reset_world', False):
                world = World(WIDTH, HEIGHT, gravity=(0, 981), friction=0.1, restitution=0.5, constraint_iterations=world.constraint_iterations)
                emitter = None
                _shared['reset_world'] = False
            if _shared.get('spawn_emitter', False):
                if emitter is None:
                    emitter = Emitter(Vec2(WIDTH // 2, HEIGHT // 2), direction=Vec2(0, -1), velocity=500, rate=0.05)
                _shared['spawn_emitter'] = False
            if _shared.get('__exit__', False):
                running = False
            # Update world parameters from GUI
            world.verlet_rebuild_freq = int(_shared.get('verlet_rebuild_freq', world.verlet_rebuild_freq))
            world.verlet_skin = float(_shared.get('verlet_skin', world.verlet_skin))
            world.collision_iterations = int(_shared.get('collision_iterations', world.collision_iterations))
            world.constraint_iterations = int(_shared.get('constraint_iterations', world.constraint_iterations))

            # Apply edits from GUI to the selected constraint (if any)
            sc = _shared.get('selected_constraint')
            if sc and isinstance(sc, dict):
                idx = sc.get('index')
                if isinstance(idx, int) and 0 <= idx < len(world.constraints):
                    c = world.constraints[idx]
                    # ensure it's still the same type
                    if type(c).__name__ == sc.get('type'):
                        # only handle springs here (defensive attribute names)
                        if sc.get('type') == 'Spring':
                            # helper to get float from shared safely
                            def _get_shared_float(key, fallback):
                                try:
                                    v = _shared.get(key, fallback)
                                    if v is None:
                                        return None
                                    return float(v)
                                except Exception:
                                    return None
                            # length/rest_length
                            new_len = _get_shared_float('spring_length', sc.get('length'))
                            if new_len is not None:
                                if hasattr(c, 'length'):
                                    try: c.length = new_len
                                    except Exception: pass
                                elif hasattr(c, 'rest_length'):
                                    try: c.rest_length = new_len
                                    except Exception: pass
                                elif hasattr(c, 'rest'):
                                    try: c.rest = new_len
                                    except Exception: pass
                            # stiffness / k
                            new_k = _get_shared_float('spring_stiffness', sc.get('stiffness'))
                            if new_k is not None:
                                if hasattr(c, 'stiffness'):
                                    try: c.stiffness = new_k
                                    except Exception: pass
                                elif hasattr(c, 'k'):
                                    try: c.k = new_k
                                    except Exception: pass
                            # damping
                            new_d = _get_shared_float('spring_damping', sc.get('damping'))
                            if new_d is not None and hasattr(c, 'damping'):
                                try: c.damping = new_d
                                except Exception: pass
                            # keep selected_constraint record in sync
                            sc['length'] = getattr(c, 'length', getattr(c, 'rest_length', sc.get('length')))
                            sc['stiffness'] = getattr(c, 'stiffness', getattr(c, 'k', sc.get('stiffness')))
                            sc['damping'] = getattr(c, 'damping', sc.get('damping'))
                            _shared['selected_constraint'] = sc
        except Exception:
            pass

        # --- Update ---
        if not paused:
            dt = 1.0 / FPS

            if not paused and emitter:
                emitter.update(dt, world)
                # Remove particles that exceeded their lifetime
                world.particles = [p for p in world.particles
                                if not hasattr(p, 'lifetime') or p.lifetime > 0]
                for p in world.particles:
                    if hasattr(p, 'lifetime'):
                        p.lifetime -= dt

            world.update(dt)

        # --- Draw ---
        screen.fill(WHITE)
                
        world.draw_particles(screen, selected_particle=selected_particle)

        world.draw_constraints(screen)
            
        cloth_solver.draw(screen) if cloth_solver else None

        if paused:
            pause_text = font.render("PAUSED", True, BLACK)
            screen.blit(pause_text, (WIDTH - pause_text.get_width() - 10, 10))

        if emitter:
            emitter.draw(screen)

        # draw particle count
        particle_count = len(world.particles)
        count_surf = font.render(f"Particles: {particle_count}", True, BLACK)
        screen.blit(count_surf, (10, HEIGHT - 30))

        pygame.display.flip()
        clock.tick(FPS)

    # cleanup: signal GUI to exit and join
    try:
        _shared['__exit__'] = True
        _gui_proc.join(timeout=1.0)
    except Exception:
        pass

    pygame.quit()

if __name__ == "__main__":
    main()
