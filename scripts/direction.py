import numpy as np

c0 = 0.2
c1 = 2.0
c2 = 0.3


def give_direction(x, p, q):
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
        speed = min(c0, dist*c0/c1)
    else:
        speed = min(c0*c2/abs(l), c0, dist*c0/c1)
    vel = np.empty(3, dtype=np.float64)
    vel[:2] = speed * dhat
    vel[2] = 0
    return error, speed, vel, dist


# p = np.array((5, 5, 10), dtype=np.float64)
# q = np.array((10, 5, 10), dtype=np.float64)
# x = np.array((12, 5, 10), dtype=np.float64)

# err, vel, dist = give_direction(x, p, q)
# print("e", err, "v", vel, "d", dist)
