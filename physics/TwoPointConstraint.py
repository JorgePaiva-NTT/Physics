import pygame

class TwoPointConstraint:
    """Base class for constraints involving two particles."""
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def set_particles(self, p1, p2):
        self.p1 = p1
        self.p2 = p2
        
    def update(self, dt, eps=1e-9):
        """Update the constraint. To be implemented by subclasses."""
        raise NotImplementedError("Subclasses should implement this method.")
    
    def __repr__(self):
        return f"<{self.__class__.__name__} p1={self.p1} p2={self.p2}>"
    
    def __str__(self):
        return self.__repr__()
    
    def compute_force(self, p1_pos, p2_pos, p1_vel, p2_vel):
        """Compute the force exerted by the constraint. To be implemented by subclasses."""
        raise NotImplementedError("Subclasses should implement this method.")
    
    def draw(self, screen, color=(0,0,0), secondary_color=(255,0,0)):
        """Helper to draw the constraint."""
        pygame.draw.line(screen, color,
                         (int(self.p1.pos.x), int(self.p1.pos.y)),
                         (int(self.p2.pos.x), int(self.p2.pos.y)), 1)