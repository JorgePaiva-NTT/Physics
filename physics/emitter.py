from .Particle import Particle
from .Vec2 import Vec2
import math
import random
import pygame

class Emitter:
    def __init__(self, pos, direction, spread=15, velocity=200, rate=0.1, 
                 particle_lifetime=5, particle_radius=10, particle_mass=1):
        self.pos = pos
        self.direction = direction.normalize()
        self.spread = spread  # Spread angle in degrees
        self.velocity = velocity
        self.rate = rate  # Time between particle emissions
        self.time_since_last = 0
        self.particle_lifetime = particle_lifetime
        self.particle_radius = particle_radius
        self.particle_mass = particle_mass
    
    def update(self, dt, world):
        self.time_since_last += dt
        if self.time_since_last >= self.rate:
            self.emit(world)
            self.time_since_last = 0
    
    def emit(self, world):
        # Calculate random spread angle
        angle = math.atan2(self.direction.y, self.direction.x)
        spread_rad = math.radians(random.uniform(-self.spread, self.spread))
        final_angle = angle + spread_rad
        
        # Create velocity vector
        vel = Vec2(
            math.cos(final_angle) * self.velocity,
            math.sin(final_angle) * self.velocity
        )
        
        # Create new particle
        particle = Particle(
            pos=Vec2(self.pos.x, self.pos.y),
            vel=vel,
            mass=self.particle_mass,
            radius=self.particle_radius
        )
        
        world.add_particle(particle)
        
    def draw(self, screen):
        try:
            pygame.draw.circle(screen, (255, 0, 0), (int(self.pos.x), int(self.pos.y)), 5)
            end_pos = self.pos + self.direction * 20
            pygame.draw.line(screen, (255, 0, 0), (int(self.pos.x), int(self.pos.y)), (int(end_pos.x), int(end_pos.y)), 2)
        except Exception:
            pass