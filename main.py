import pygame
from physics.world import World
from physics.core import Particle, Vec2
from physics.constraints import DistanceConstraint, PinConstraint
from physics.spring import Spring
from physics.serialization import save_world, load_world

# --- Constants ---
WIDTH, HEIGHT = 800, 600
FPS = 60

# --- Colors ---
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
MAGENTA = (255, 0, 255)

# --- Pygame Setup ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Physics Playground")
clock = pygame.time.Clock()

# --- Physics World ---
world = World(WIDTH, HEIGHT, gravity=Vec2(0, 981), friction=0.1)

# --- Simulation Setup ---
p1 = Particle(Vec2(300, 200))
p2 = Particle(Vec2(400, 200))
world.add_particle(p1)
world.add_particle(p2)
world.add_constraint(DistanceConstraint(p1, p2, 100))


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
            if event.button == 1: # Left click
                clicked_on_particle = False
                for p in world.particles:
                    if (Vec2(event.pos[0], event.pos[1]) - p.pos).length() < p.radius:
                        if first_particle_for_constraint is None:
                            first_particle_for_constraint = p
                        else:
                            if first_particle_for_constraint != p:
                                 world.add_constraint(DistanceConstraint(first_particle_for_constraint, p, (first_particle_for_constraint.pos - p.pos).length()))
                                 first_particle_for_constraint = None
                                 selected_particle = p
                            elif event.key == pygame.K_f: # 'f' key for fixed
                                 p.fixed = not p.fixed
                                 break
                            elif event.key == pygame.K_b: # 'b' key for pin
                                world.add_constraint(PinConstraint(p, p.pos))
                                break
                            elif event.key == pygame.K_s: # 's' key for spring
                                 if first_particle_for_spring is None:
                                     first_particle_for_spring = p
                        first_particle_for_constraint = None
                        selected_particle = p
                        clicked_on_particle = True
                        break
                if not clicked_on_particle:
                    selected_particle = None
                    first_particle_for_constraint = None
            elif event.button == 3: # Right click
                new_particle = Particle(Vec2(event.pos[0], event.pos[1]))
                world.add_particle(new_particle)
        elif event.type == pygame.MOUSEBUTTONUP:
            selected_particle = None
        elif event.type == pygame.MOUSEMOTION:
            if selected_particle:
                selected_particle.pos = Vec2(event.pos[0], event.pos[1])
        elif event.type == pygame.KEYDOWN:
            if input_gravity_mode:
                if event.key == pygame.K_RETURN:
                    try:
                        new_gravity_y = float(input_gravity_text)
                        world.gravity = Vec2(0, new_gravity_y)
                    except ValueError:
                        print("Invalid gravity value")
                    input_gravity_mode = False
                    input_gravity_text = ''
                elif event.key == pygame.K_BACKSPACE:
                    input_gravity_text = input_gravity_text[:-1]
                else:
                    input_gravity_text += event.unicode
            elif input_mass_mode:
                if event.key == pygame.K_RETURN:
                    try:
                        new_mass = float(input_mass_text)
                        if particle_to_change_mass:
                            particle_to_change_mass.mass = new_mass
                    except ValueError:
                        print("Invalid mass value")
                    input_mass_mode = False
                    input_mass_text = ''
                    particle_to_change_mass = None
                elif event.key == pygame.K_BACKSPACE:
                    input_mass_text = input_mass_text[:-1]
                else:
                    input_mass_text += event.unicode
            elif input_radius_mode:
                if event.key == pygame.K_RETURN:
                    try:
                        new_radius = float(input_radius_text)
                        if particle_to_change_radius:
                            particle_to_change_radius.radius = new_radius
                    except ValueError:
                        print("Invalid radius value")
                    input_radius_mode = False
                    input_radius_text = ''
                    particle_to_change_radius = None
                elif event.key == pygame.K_BACKSPACE:
                    input_radius_text = input_radius_text[:-1]
                else:
                    input_radius_text += event.unicode
            elif input_stiffness_mode:
                if event.key == pygame.K_RETURN:
                    try:
                        new_stiffness = float(input_stiffness_text)
                        if spring_to_change_stiffness:
                            spring_to_change_stiffness.stiffness = new_stiffness
                    except ValueError:
                        print("Invalid stiffness value")
                    input_stiffness_mode = False
                    input_stiffness_text = ''
                    spring_to_change_stiffness = None
                elif event.key == pygame.K_BACKSPACE:
                    input_stiffness_text = input_stiffness_text[:-1]
                else:
                    input_stiffness_text += event.unicode
            elif input_length_mode:
                if event.key == pygame.K_RETURN:
                    try:
                        new_length = float(input_length_text)
                        if spring_to_change_length:
                            spring_to_change_length.length = new_length
                    except ValueError:
                        print("Invalid length value")
                    input_length_mode = False
                    input_length_text = ''
                    spring_to_change_length = None
                elif event.key == pygame.K_BACKSPACE:
                    input_length_text = input_length_text[:-1]
                else:
                    input_length_text += event.unicode
            else:
                if event.key == pygame.K_c: # 'c' key for clear
                    world.particles.clear()
                    world.constraints.clear()
                    selected_particle = None
                    first_particle_for_constraint = None
                    first_particle_for_spring = None
                elif event.key == pygame.K_g: # 'g' key for gravity
                    input_gravity_mode = True
                    input_gravity_text = str(world.gravity.y)
                elif event.key == pygame.K_p: # 'p' key for pause
                    paused = not paused
                elif event.key == pygame.K_k: # 'k' key for save
                    save_world(world, "simulation_state.json")
                    print("Simulation state saved to simulation_state.json")
                elif event.key == pygame.K_l: # 'l' key for load
                    try:
                        world = load_world("simulation_state.json")
                        print("Simulation state loaded from simulation_state.json")
                    except FileNotFoundError:
                        print("No saved simulation state found.")
                    selected_particle = None
                    first_particle_for_constraint = None
                    first_particle_for_spring = None
                else:
                    mouse_pos = Vec2(pygame.mouse.get_pos()[0], pygame.mouse.get_pos()[1])
                    for p in world.particles:
                        if (mouse_pos - p.pos).length() < p.radius:
                            if event.key == pygame.K_d: # 'd' key for delete
                                world.remove_particle(p)
                                if selected_particle == p:
                                    selected_particle = None
                                if first_particle_for_constraint == p:
                                    first_particle_for_constraint = None
                                if first_particle_for_spring == p:
                                    first_particle_for_spring = None
                                break
                            elif event.key == pygame.K_f: # 'f' key for fixed
                                p.fixed = not p.fixed
                                break
                            elif event.key == pygame.K_s: # 's' key for spring
                                if first_particle_for_spring is None:
                                    first_particle_for_spring = p
                                else:
                                    if first_particle_for_spring != p:
                                        world.add_constraint(Spring(first_particle_for_spring, p, (first_particle_for_spring.pos - p.pos).length(), 20))
                                        first_particle_for_spring = None
                                break
                            elif event.key == pygame.K_m: # 'm' key for mass
                                input_mass_mode = True
                                input_mass_text = str(p.mass)
                                particle_to_change_mass = p
                                break
                            elif event.key == pygame.K_r: # 'r' key for radius
                                input_radius_mode = True
                                input_radius_text = str(p.radius)
                                particle_to_change_radius = p
                                break
                        if event.key == pygame.K_t: # 't' key for stiffness
                            for c in world.constraints:
                                if isinstance(c, Spring):
                                    # Check if mouse is near the spring (midpoint)
                                    mid_x = (c.p1.pos.x + c.p2.pos.x) / 2
                                    mid_y = (c.p1.pos.y + c.p2.pos.y) / 2
                                    if (Vec2(mouse_pos.x, mouse_pos.y) - Vec2(mid_x, mid_y)).length() < 15: # Small tolerance
                                        input_stiffness_mode = True
                                        input_stiffness_text = str(c.stiffness)
                                        spring_to_change_stiffness = c
                                        break
                            break
                        elif event.key == pygame.K_y: # 'y' key for length
                            for c in world.constraints:
                                if isinstance(c, Spring):
                                    # Check if mouse is near the spring (midpoint)
                                    mid_x = (c.p1.pos.x + c.p2.pos.x) / 2
                                    mid_y = (c.p1.pos.y + c.p2.pos.y) / 2
                                    if (Vec2(mouse_pos.x, mouse_pos.y) - Vec2(mid_x, mid_y)).length() < 15: # Small tolerance
                                        input_length_mode = True
                                        input_length_text = str(c.length)
                                        spring_to_change_length = c
                                        break
                                break

    # --- Update ---
    if not paused:
        dt = 1.0 / FPS
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

    pygame.display.flip()

    clock.tick(FPS)

pygame.quit()