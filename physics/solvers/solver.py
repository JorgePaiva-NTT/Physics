class solver:
    def __init__(self, indices, fixed=False):
        self.fixed = bool(fixed)
        self.indices = list(indices)

    def solve(self, world, dt):
        """
        Placeholder for a solver.
        """
        pass
    
    def draw(self):
        """
        Placeholder for a solver draw method.
        """
        pass
    
    def __repr__(self):
        return f"<{self.__class__.__name__} fixed={self.fixed} indices={self.indices}>"
    
    def __str__(self):
        return self.__repr__()