
import pygame
from physics.world import World
from physics.core import Particle, Vec2
from physics.spring import Spring

# --- Constants ---
WIDTH, HEIGHT = 800, 600
FPS = 60

# --- Colors ---
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

# --- Pygame Setup ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Spring Example")
clock = pygame.time.Clock()

# --- Physics World ---
world = World(WIDTH, HEIGHT)

# --- Simulation Setup ---
p1 = Particle(Vec2(300, 200))
p2 = Particle(Vec2(400, 200))
world.add_particle(p1)
world.add_particle(p2)
world.add_constraint(Spring(p1, p2, 100, 0.1))

# --- Main Loop ---
running = True
selected_particle = None
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            for p in world.particles:
                if (Vec2(event.pos[0], event.pos[1]) - p.pos).length() < p.radius:
                    selected_particle = p
                    break
        elif event.type == pygame.MOUSEBUTTONUP:
            selected_particle = None
        elif event.type == pygame.MOUSEMOTION:
            if selected_particle:
                selected_particle.pos = Vec2(event.pos[0], event.pos[1])

    # --- Update ---
    dt = 1.0 / FPS
    world.update(dt)

    # --- Draw ---
    screen.fill(WHITE)

    for p in world.particles:
        pygame.draw.circle(screen, RED, (int(p.pos.x), int(p.pos.y)), p.radius)

    for c in world.constraints:
        pygame.draw.line(screen, BLACK, (int(c.p1.pos.x), int(c.p1.pos.y)), (int(c.p2.pos.x), int(c.p2.pos.y)), 2)

    pygame.display.flip()

    clock.tick(FPS)

pygame.quit()
