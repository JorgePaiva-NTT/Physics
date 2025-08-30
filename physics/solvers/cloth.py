from physics.DistanceConstraint import DistanceConstraint
from .solver import solver
from ..TwoPointConstraint import TwoPointConstraint

class Cloth(solver):
    def __init__(self, particle_indices, width, height, tear_distance_factor=0, stiffness=0.5):
        """
        A cloth solver that uses distance constraints to simulate a grid of particles.
        """
        super().__init__(particle_indices)
        self.width = width
        self.height = height
        self.stiffness = stiffness
        self.tear_distance_factor = tear_distance_factor
        self.constraints: list[TwoPointConstraint] = []

    def build_constraints(self, world):
        """
        Builds the internal distance constraints for the cloth grid.
        This should be called after the particles have been added to the world.
        """
        self.constraints = []

        k_prime = self.stiffness
        if world.constraint_iterations > 0:
            k_prime = 1.0 - (1.0 - self.stiffness) ** (1.0 / world.constraint_iterations)

        particles = [world.particles[i] for i in self.indices]

        def get_particle(x, y):
            if 0 <= x < self.width and 0 <= y < self.height:
                return particles[y * self.width + x]
            return None

        for y in range(self.height):
            for x in range(self.width):
                p1 = get_particle(x, y)
                
                # Structural constraints (horizontal and vertical)
                for dx, dy in [(1, 0), (0, 1)]:
                    p2 = get_particle(x + dx, y + dy)
                    if p2:
                        length = (p1.pos - p2.pos).length() 
                        constraint = DistanceConstraint(p1, p2, length, stiffness=k_prime)
                        self.constraints.append(constraint)
                
                # Shear constraints (diagonals) for more rigidity
                for dx, dy in [(1, 1), (-1, 1)]:
                    p2 = get_particle(x + dx, y + dy)
                    if p2:
                        length = (p1.pos - p2.pos).length()
                        constraint = DistanceConstraint(p1, p2, length, stiffness=k_prime)
                        self.constraints.append(constraint)

    def solve(self, world, dt):
        """
        Solves the internal distance constraints and handles tearing logic.
        This method is called on each iteration of the world's constraint solving loop.
        """
        if self.fixed:
            return

        # Solve all internal distance constraints
        for c in self.constraints:
            c.update(dt)

        # Check for tearing. If a spring tears, remove it from the cloth's
        # internal list and also from the main world simulation.
        if self.tear_distance_factor > 0:
            torn_constraints = []
            for c in self.constraints:
                current_length = (c.p1.pos - c.p2.pos).length()
                if current_length >= c.distance * self.tear_distance_factor:
                    torn_constraints.append(c)
            
            if torn_constraints:
                self.constraints = [c for c in self.constraints if c not in torn_constraints]
                # The constraints are managed internally, so no need to modify world.constraints

    def draw(self, screen):
        """Helper to draw the cloth constraints."""
        c: TwoPointConstraint
        for c in self.constraints:
            c.draw(screen, (0, 0, 0), (255, 0, 0))
