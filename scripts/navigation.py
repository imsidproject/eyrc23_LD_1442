import numpy as np


class Navigation():
    def __init__(self):
        self.c0 = 0.2
        self.c1 = 2.0
        self.c2 = 0.3
        ht = 20
        self.setpoints = np.array(((-7, -7, ht), (7, -7, ht), (7, -3.5, ht), (-7, -3.5, ht), (-7, 0, ht),
                                  (7, 0, ht), (7, 3.5, ht), (-7, 3.5, ht), (-7, 7, ht), (7, 7, ht)), dtype=np.float64)
        self.stpt_i = 0

    def get_direction(self, x):
        error, vel, dist = self.calc_error(
            x, self.setpoints[self.stpt_i], self.setpoints[self.stpt_i + 1])
        if dist <= 0:
            self.stpt_i += 1
        return error, vel

    def get_start_point(self):
        return self.setpoints[0]

    def calc_error(self, x, p, q):
        dv = q[:2]-p[:2]
        nv = np.float64((-dv[1], dv[0]))
        nmod = np.linalg.norm(nv)
        nhat = nv/nmod
        dhat = dv/nmod
        error = np.empty(3, dtype=np.float64)
        l = sum((p-x)[:2]*nhat[:])
        error[:2] = nhat*l
        error[2] = q[2]-x[2]
        dist = np.linalg.norm(dv) - sum((x-p)[:2]*dhat)
        if l == 0:
            speed = min(self.c0, dist*self.c0/self.c1)
        else:
            speed = min(self.c0*self.c2/abs(l), self.c0, dist*self.c0/self.c1)
        vel = np.empty(3, dtype=np.float64)
        vel[:2] = speed * dhat
        vel[2] = 0
        return error, vel, dist


# p = np.array((5, 5, 10), dtype=np.float64)
# q = np.array((10, 5, 10), dtype=np.float64)
# x = np.array((12, 5, 10), dtype=np.float64)

# err, vel, dist = give_direction(x, p, q)
# print("e", err, "v", vel, "d", dist)
