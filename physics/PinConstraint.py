from physics.Vec2 import Vec2
import pygame


class PinConstraint:
    def __init__(self, p, pos):
        self.p = p
        # store a Vec2 copy of pinned location
        self.pos = pos.copy() if hasattr(pos, "copy") else Vec2(pos.x, pos.y)
        # mark particle fixed for RK4 integrator so solver treats it immovable
        try:
            self.p.fixed = True
        except Exception:
            pass

    def update(self):
        # enforce pinned position each frame (positional correction)
        try:
            self.p.pos = self.pos.copy()
            if hasattr(self.p, "vel"):
                self.p.vel = Vec2(0.0, 0.0)
        except Exception:
            pass

    def set_particle(self, p):
        # un-fix previous particle and fix new one
        try:
            if hasattr(self, "p") and self.p is not None:
                self.p.fixed = False
        except Exception:
            pass
        self.p = p
        try:
            self.p.fixed = True
        except Exception:
            pass
        
    def draw(self, screen, color=(0,0,0)):
        try:
            pygame.draw.circle(screen, color, (int(self.pos.x), int(self.pos.y)), 5)
            if hasattr(self, "p") and self.p is not None:
                pygame.draw.line(screen, color, (int(self.pos.x), int(self.pos.y)), (int(self.p.pos.x), int(self.p.pos.y)), 1)
        except Exception:
            pass