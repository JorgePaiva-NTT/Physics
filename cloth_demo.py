import pygame
from physics.world import World
from physics.core import Particle, Vec2
from physics.spring import Spring
from physics.constraints import PinConstraint

WIDTH, HEIGHT = 800, 600
FPS = 60
# use a smaller physics timestep for stability (substep)
DT = 1.0 / 120.0

def create_cloth(world, origin, cols, rows, spacing, stiffness, spring_damping=1.2):
    particles = [[None for _ in range(cols)] for _ in range(rows)]
    ox, oy = origin.x, origin.y

    # create particles
    for r in range(rows):
        for c in range(cols):
            pos = Vec2(ox + c * spacing, oy + r * spacing)
            p = Particle(pos)                      # uses default radius/mass
            world.add_particle(p)
            particles[r][c] = p

    # structural springs (horizontal + vertical)
    for r in range(rows):
        for c in range(cols):
            p = particles[r][c]
            if c < cols - 1:
                right = particles[r][c + 1]
                s = Spring(p, right, (p.pos - right.pos).length(), stiffness)
                s.damping = spring_damping
                world.add_constraint(s)
            if r < rows - 1:
                down = particles[r + 1][c]
                s = Spring(p, down, (p.pos - down.pos).length(), stiffness)
                s.damping = spring_damping
                world.add_constraint(s)

    # shear springs (diagonals)
    for r in range(rows - 1):
        for c in range(cols - 1):
            p = particles[r][c]
            diag1 = particles[r + 1][c + 1]
            diag2 = particles[r + 1][c]
            right = particles[r][c + 1]
            s = Spring(p, diag1, (p.pos - diag1.pos).length(), stiffness * 0.9)
            s.damping = spring_damping
            world.add_constraint(s)
            s = Spring(right, diag2, (right.pos - diag2.pos).length(), stiffness * 0.9)
            s.damping = spring_damping
            world.add_constraint(s)

    # pin top row
    for c in range(cols):
        top = particles[0][c]
        world.add_constraint(PinConstraint(top, Vec2(top.pos.x, top.pos.y)))

def draw(world, screen):
    # draw springs
    for c in world.constraints:
        # Spring instances have p1/p2
        if hasattr(c, "p1") and hasattr(c, "p2") and type(c).__name__ == "Spring":
            pygame.draw.line(screen, (0, 0, 0),
                             (int(c.p1.pos.x), int(c.p1.pos.y)),
                             (int(c.p2.pos.x), int(c.p2.pos.y)), 2)
    # draw particles
    for p in world.particles:
        pygame.draw.circle(screen, (200, 30, 30), (int(p.pos.x), int(p.pos.y)), getattr(p, "radius", 5))

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    clock = pygame.time.Clock()
    # reduced friction and restitution for stability
    world = World(WIDTH, HEIGHT, gravity=Vec2(0, 981), friction=0.02, restitution=0.3)

    # lower stiffness (try 200) and add per-spring damping
    create_cloth(world, Vec2(200, 80), cols=6, rows=6, spacing=30, stiffness=200.0, spring_damping=1.2)

    running = True
    paused = False

    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                paused = not paused

        if not paused:
            world.update(DT)

        screen.fill((255, 255, 255))
        draw(world, screen)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()

if __name__ == "__main__":
    main()