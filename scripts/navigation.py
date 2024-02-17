import numpy as np


class Navigation():
    def __init__(self):
        self.c0 = 0.4
        self.c1 = 2.0
        self.c2 = 0.3
        self.ht = 20
        ht = self.ht
        self.setpoints = np.array(((-1, -1, -1), (-7, -7, ht), (7, -7, ht), (7, -3.5, ht), (-7, -3.5, ht), (-7, 0, ht),
                                  (7, 0, ht), (7, 3.5, ht), (-7, 3.5, ht), (-7, 7, ht), (7, 7, ht)), dtype=np.float64)
        self.stpt_i = -1
        self.vel = np.zeros(3, np.float64)
        self.error = np.empty(3, dtype=np.float64)
        self.base_ht = 30

    def get_direction(self, x):
        if self.setpoints[0, 2] == -1:
            self.setpoints[0, :] = x
            self.setpoints[0, 2] = self.ht
            self.base_ht = x[2]
        if self.stpt_i == -1:
            return self.take_off(x)
        if self.stpt_i >= len(self.setpoints)-1:
            return self.land(x)
        error, vel, dist = self.calc_error(
            x, self.setpoints[self.stpt_i], self.setpoints[self.stpt_i + 1])
        if dist <= 0:
            self.stpt_i += 1
        return error, vel

    def get_start_point(self):
        return self.setpoints[0]

    def take_off(self, x):
        self.error[:] = self.setpoints[0] - x
        self.vel.fill(0)
        if np.bitwise_and(self.error < 0.3, self.error > -0.3).all():
            self.stpt_i = 0
        return self.error, self.vel

    def land(self, x):
        self.error[:] = self.setpoints[-1] - x
        self.error[2] = 0
        self.vel.fill(0)
        self.vel[2] = 0.01 * (self.base_ht - x[2])
        if self.base_ht - x[2] < 0.2:
            return None, None
        return self.error, self.vel

    def calc_error(self, x, p, q):
        dv = q[:2]-p[:2]
        nv = np.float64((-dv[1], dv[0]))
        nmod = np.linalg.norm(nv)
        nhat = nv/nmod
        dhat = dv/nmod

        l = sum((p-x)[:2]*nhat[:])
        self.error[:2] = nhat*l
        self.error[2] = q[2]-x[2]
        dist = np.linalg.norm(dv) - sum((x-p)[:2]*dhat)
        if l == 0:
            speed = min(self.c0, dist*self.c0/self.c1)
        else:
            speed = min(self.c0*self.c2/abs(l), self.c0, dist*self.c0/self.c1)

        self.vel[:2] = speed * dhat
        self.vel[2] = 0
        return self.error, self.vel, dist


# p = np.array((5, 5, 10), dtype=np.float64)
# q = np.array((10, 5, 10), dtype=np.float64)
# x = np.array((12, 5, 10), dtype=np.float64)

# err, vel, dist = give_direction(x, p, q)
# print("e", err, "v", vel, "d", dist)
