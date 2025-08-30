import math
from .solver import solver
from ..Vec2 import Vec2

class RigidBody(solver):
    def __init__(self, indices, rest_rel, stiffness=1.0, fixed=False):
        super().__init__(indices, fixed)
        self.rest_rel = [r.copy() for r in rest_rel]
        self.stiffness = float(stiffness)
        self.rest_com = Vec2(0.0, 0.0)
            
    def solve(self, world, dt):
        if (len(self.indices) == 0) or self.fixed:
            return
        ps = [world.particles[i] for i in self.indices]
        if len(ps) == 1:
            if self.fixed:
                p = ps[0]
                p.fixed = True
            return
        
        com_x = 0.0; com_y = 0.0
        count = 0
        for p in ps:
            com_x += p.pos.x
            com_y += p.pos.y
            count += 1
        if count == 0:
            return
        com_x /= count
        com_y /= count
        com = Vec2(com_x, com_y)
        self.rest_com = com
        
        a = 0.0
        b = 0.0
        for p, r in zip(ps, self.rest_rel):
            px = p.pos.x - com.x
            py = p.pos.y - com.y
            rx = r.x
            ry = r.y
            a += px * rx + py * ry
            b += px * ry - py * rx
            
        if a == 0.0 and b == 0.0:
            cos_t = 1.0
            sin_t = 0.0
        else:
            theta = math.atan2(b, a)
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
        
        # apply shape-matching projection with stiffness
        for idx, (p, r) in enumerate(zip(ps, self.rest_rel)):
            if getattr(p, 'fixed', False):
                continue
            
            tx = com.x + (cos_t * r.x - sin_t * r.y)
            ty = com.y + (sin_t * r.x + cos_t * r.y)
            
            alpha = max(0.0, min(1.0, self.stiffness))
            new_x = p.pos.x + alpha * (tx - p.pos.x)
            new_y = p.pos.y + alpha * (ty - p.pos.y)
            
            p.pos = Vec2(new_x, new_y)
