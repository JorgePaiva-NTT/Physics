import pygame
from physics.world import World
from physics.core import Particle, Vec2
from physics.constraints import DistanceConstraint, PinConstraint
from physics.spring import Spring
from physics.serialization import save_world, load_world
from physics.emitter import Emitter

# --- Constants ---
WIDTH, HEIGHT = 800, 600
FPS = 60
DT = 1.0 / FPS

# --- Colors ---
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
MAGENTA = (255, 0, 255)

EMITTER_COLOR = (128, 0, 128)  # Purple for emitter
EMITTER_SIZE = 20

# --- Pygame Setup ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Physics Playground")
clock = pygame.time.Clock()

# --- Physics World ---
world = World(WIDTH, HEIGHT, gravity=Vec2(0, 981), friction=0.1, restitution=0.5)

# --- Simulation Setup ---
p1 = Particle(Vec2(300, 200))
p2 = Particle(Vec2(400, 200))
world.add_particle(p1)
world.add_particle(p2)
world.add_constraint(Spring(p1, p2, (p1.pos - p2.pos).length(), 15))  # Changed from 25

emitter = None
input_emitter_rate_mode = False
input_emitter_rate_text = ''

# --- Main Loop ---
running = True
selected_particle = None
first_particle_for_constraint = None
first_particle_for_spring = None
input_gravity_mode = False
input_gravity_text = ''
input_mass_mode = False
input_mass_text = ''
particle_to_change_mass = None
input_radius_mode = False
input_radius_text = ''
particle_to_change_radius = None
input_stiffness_mode = False
input_stiffness_text = ''
spring_to_change_stiffness = None
input_length_mode = False
input_length_text = ''
spring_to_change_length = None
global_friction = None
paused = False

font = pygame.font.Font(None, 32)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = event.pos
            # Left-click: select/add particle or create spring between two clicked particles
            if event.button == 1:
                clicked = None
                # find particle under cursor (first match)
                for p in world.particles:
                    dx = p.pos.x - mx
                    dy = p.pos.y - my
                    if (dx * dx + dy * dy) <= (p.radius + 3) ** 2:
                        clicked = p
                        break
                if clicked:
                    # If user is selecting a pair to create a spring
                    if first_particle_for_spring is None:
                        first_particle_for_spring = clicked
                    else:
                        if clicked is not first_particle_for_spring:
                            # create spring between first and clicked
                            rest_len = (first_particle_for_spring.pos - clicked.pos).length()
                            world.add_constraint(Spring(first_particle_for_spring, clicked, rest_len, 15))
                        first_particle_for_spring = None
                else:
                    # create a new particle at mouse pos
                    world.add_particle(Particle(Vec2(mx, my)))
            # Right-click: toggle emitter at click position
            elif event.button == 3:
                if emitter is None:
                    emitter = Emitter(Vec2(mx, my), direction=Vec2(0, -1), velocity=500, rate=0.05)
                else:
                    emitter = None
        elif event.type == pygame.MOUSEBUTTONUP:
            pass
        elif event.type == pygame.MOUSEMOTION:
            pass
        elif event.type == pygame.KEYDOWN:
            # Tuning keys (runtime)
            # Verlet rebuild frequency: '[' to decrease, ']' to increase
            if event.key == pygame.K_c:
                world.verlet_rebuild_freq = max(1, int(world.verlet_rebuild_freq) - 1)
            elif event.key == pygame.K_v:
                world.verlet_rebuild_freq = int(world.verlet_rebuild_freq) + 1
            # Verlet skin: '-' to decrease, '=' to increase
            elif event.key == pygame.K_s:
                world.verlet_skin = max(0.0, float(world.verlet_skin) - 0.5)
            elif event.key == pygame.K_d:
                world.verlet_skin = float(world.verlet_skin) + 0.5
            # Collision iterations: ',' to decrease, '.' to increase
            elif event.key == pygame.K_i:
                world.collision_iterations = max(1, int(world.collision_iterations) - 1)
            elif event.key == pygame.K_o:
                world.collision_iterations = int(world.collision_iterations) + 1
            # Pause toggle (keep existing control)
            elif event.key == pygame.K_SPACE:
                paused = not paused
            # keep other key handling (inputs) as-is
            else:
                pass
        elif event.type == pygame.KEYUP:
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

    for p in world.particles:
        color = RED
        if p.fixed:
            color = YELLOW
        elif p == selected_particle:
            color = BLUE
        elif p == first_particle_for_constraint:
            color = GREEN
        elif p == first_particle_for_spring:
            color = MAGENTA
        pygame.draw.circle(screen, color, (int(p.pos.x), int(p.pos.y)), p.radius)

    for c in world.constraints:
        pygame.draw.line(screen, BLACK, (int(c.p1.pos.x), int(c.p1.pos.y)), (int(c.p2.pos.x), int(c.p2.pos.y)), 2)

    if input_gravity_mode:
        text_surface = font.render(f"Enter gravity (Y): {input_gravity_text}", True, BLACK)
        screen.blit(text_surface, (10, 10))
    elif input_mass_mode:
        text_surface = font.render(f"Enter mass: {input_mass_text}", True, BLACK)
        screen.blit(text_surface, (10, 10))
    elif input_radius_mode:
        text_surface = font.render(f"Enter radius: {input_radius_text}", True, BLACK)
        screen.blit(text_surface, (10, 10))
    elif input_stiffness_mode:
        text_surface = font.render(f"Enter stiffness: {input_stiffness_text}", True, BLACK)
        screen.blit(text_surface, (10, 10))
    elif input_length_mode:
        text_surface = font.render(f"Enter length: {input_length_text}", True, BLACK)
        screen.blit(text_surface, (10, 10))

    if paused:
        pause_text = font.render("PAUSED", True, BLACK)
        screen.blit(pause_text, (WIDTH - pause_text.get_width() - 10, 10))

    if emitter:
        # Draw emitter
        pygame.draw.circle(screen, EMITTER_COLOR, (int(emitter.pos.x), int(emitter.pos.y)), EMITTER_SIZE)
        end_pos = (int(emitter.pos.x + emitter.direction.x * EMITTER_SIZE * 2),
                   int(emitter.pos.y + emitter.direction.y * EMITTER_SIZE * 2))
        pygame.draw.line(screen, EMITTER_COLOR, (int(emitter.pos.x), int(emitter.pos.y)), end_pos, 3)

    # draw collision tuning readouts
    tuning_lines = [
        f"verlet_rebuild_freq: {world.verlet_rebuild_freq}",
        f"verlet_skin: {world.verlet_skin:.2f}",
        f"collision_iterations: {world.collision_iterations}"
    ]
    for idx, line in enumerate(tuning_lines):
        txt = font.render(line, True, BLACK)
        screen.blit(txt, (10, 10 + idx * 20))

    # draw particle count
    particle_count = len(world.particles)
    count_surf = font.render(f"Particles: {particle_count}", True, BLACK)
    screen.blit(count_surf, (10, HEIGHT - 30))

    pygame.display.flip()

    clock.tick(FPS)

pygame.quit()
