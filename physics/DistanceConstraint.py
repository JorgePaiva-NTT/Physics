from physics.TwoPointConstraint import TwoPointConstraint


class DistanceConstraint(TwoPointConstraint):
    def __init__(self, p1, p2, distance, stiffness=1.0):
        super().__init__(p1, p2)
        self.distance = float(distance)
        self.stiffness = float(stiffness)
        # compliance=0 means classic PBD is used by default, which uses the stiffness parameter.
        # To use XPBD, set compliance to a small positive value (e.g., 1e-6 for a very stiff constraint).
        # For XPBD, physical stiffness is k=1/compliance, and the 'stiffness' parameter is ignored.
        self.compliance = 0.0
        self.lagrange = 0.0  # XPBD accumulator
        
    def update(self, dt, eps=1e-9):
        pi = self.p1.pos
        pj = self.p2.pos
        wi = self.p1.inv_mass
        wj = self.p2.inv_mass

        d = pj - pi
        len_d = d.length()
        if len_d < eps or (wi + wj) == 0.0:
            return
        n = d / len_d
        C = len_d - self.distance
        wsum = wi + wj
        
        if self.compliance > 0.0:
            # ---------- XPBD ----------
            gamma = self.compliance / (dt * dt)
            d_lambda = -(C + gamma * self.lagrange) / (wsum + gamma)
            dp_i = -wi * d_lambda * n
            dp_j = +wj * d_lambda * n
            self.p1.pos += dp_i
            self.p2.pos += dp_j
            self.lagrange += d_lambda
        else:   
            # ---------- Classic PBD ----------
            s = self.stiffness * C / wsum
            self.p1.pos += wi * s * n
            self.p2.pos -= wj * s * n