import torch
import time


#   Link 1
@torch.jit.script
def p1(q, p, a):
    res = torch.hstack([a[0] - torch.sin(q[0])*p[1] + torch.cos(q[0])*p[0],
    torch.sin(q[0])*p[0] + torch.cos(q[0])*p[1],
    p[2]])
    return res

@torch.jit.script
def d1(q, y, p, a):
    res = torch.sqrt((p[2] - y[2])**2 + (torch.sin(q[0])*p[0] + torch.cos(q[0])*p[1] - y[1])**2 + (a[0] - torch.sin(q[0])*p[1] + torch.cos(q[0])*p[0] - y[0])**2)
    return res


@torch.jit.script
def r1(q, y, p, a):
    res = (-a[0]*torch.sin(q[0])*p[0] - a[0]*torch.cos(q[0])*p[1] + torch.sin(q[0])*p[0]*y[0] + torch.sin(q[0])*p[1]*y[1] - torch.cos(q[0])*p[0]*y[1] + torch.cos(q[0])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (torch.sin(q[0])*p[0] + torch.cos(q[0])*p[1] - y[1])**2 + (a[0] - torch.sin(q[0])*p[1] + torch.cos(q[0])*p[0] - y[0])**2)
    return torch.hstack((res, torch.tensor([0, 0, 0, 0, 0, 0], device=q.device)))


#   Link 2
@torch.jit.script
def p2(q, p, a):
    res = torch.hstack([a[0] + a[1]*torch.cos(q[0]) - torch.sin(q[0] + q[1])*p[1] + torch.cos(q[0] + q[1])*p[0],
    a[1]*torch.sin(q[0]) + torch.sin(q[0] + q[1])*p[0] + torch.cos(q[0] + q[1])*p[1],
    p[2]])
    return res


@torch.jit.script
def d2(q, y, p, a):
    res = torch.sqrt((p[2] - y[2])**2 + (a[1]*torch.sin(q[0]) + torch.sin(q[0] + q[1])*p[0] + torch.cos(q[0] + q[1])*p[1] - y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) + torch.sin(q[0] + q[1])*p[1] - torch.cos(q[0] + q[1])*p[0] + y[0])**2)
    return res


@torch.jit.script
def r2(q, y, p, a):
    res = torch.hstack([(-a[0]*a[1]*torch.sin(q[0]) - a[0]*torch.sin(q[0] + q[1])*p[0] - a[0]*torch.cos(q[0] + q[1])*p[1] + a[1]*torch.sin(q[0])*y[0] - a[1]*torch.cos(q[0])*y[1] + torch.sin(q[0] + q[1])*p[0]*y[0] + torch.sin(q[0] + q[1])*p[1]*y[1] - torch.cos(q[0] + q[1])*p[0]*y[1] + torch.cos(q[0] + q[1])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (a[1]*torch.sin(q[0]) + torch.sin(q[0] + q[1])*p[0] + torch.cos(q[0] + q[1])*p[1] - y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) + torch.sin(q[0] + q[1])*p[1] - torch.cos(q[0] + q[1])*p[0] + y[0])**2),
    (-a[0]*torch.sin(q[0] + q[1])*p[0] - a[0]*torch.cos(q[0] + q[1])*p[1] - a[1]*torch.sin(q[1])*p[0] - a[1]*torch.cos(q[1])*p[1] + torch.sin(q[0] + q[1])*p[0]*y[0] + torch.sin(q[0] + q[1])*p[1]*y[1] - torch.cos(q[0] + q[1])*p[0]*y[1] + torch.cos(q[0] + q[1])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (a[1]*torch.sin(q[0]) + torch.sin(q[0] + q[1])*p[0] + torch.cos(q[0] + q[1])*p[1] - y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) + torch.sin(q[0] + q[1])*p[1] - torch.cos(q[0] + q[1])*p[0] + y[0])**2)])
    return torch.hstack((res, torch.tensor([0, 0, 0, 0, 0], device=q.device)))


# Link 3
@torch.jit.script
def p3(q, p, a):
    res = torch.hstack([a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) - torch.sin(q[0] + q[1] + q[2])*p[1] + torch.cos(q[0] + q[1] + q[2])*p[0],
    a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + torch.sin(q[0] + q[1] + q[2])*p[0] + torch.cos(q[0] + q[1] + q[2])*p[1],
    p[2]])
    return res


@torch.jit.script
def d3(q, y, p, a):
    res = torch.sqrt((p[2] - y[2])**2 + (a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + torch.sin(q[0] + q[1] + q[2])*p[0] + torch.cos(q[0] + q[1] + q[2])*p[1] - y[1])**2 + (a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) - torch.sin(q[0] + q[1] + q[2])*p[1] + torch.cos(q[0] + q[1] + q[2])*p[0] - y[0])**2)
    return res


@torch.jit.script
def r3(q, y, p, a):
    res = torch.hstack([(-a[0]*a[1]*torch.sin(q[0]) - a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*torch.sin(q[0] + q[1] + q[2])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2])*p[1] + a[1]*torch.sin(q[0])*y[0] - a[1]*torch.cos(q[0])*y[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + torch.sin(q[0] + q[1] + q[2])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - torch.sin(q[0] + q[1] + q[2])*p[0] - torch.cos(q[0] + q[1] + q[2])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) + torch.sin(q[0] + q[1] + q[2])*p[1] - torch.cos(q[0] + q[1] + q[2])*p[0] + y[0])**2),
    (-a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*torch.sin(q[0] + q[1] + q[2])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2])*p[1] - a[1]*a[2]*torch.sin(q[1]) - a[1]*torch.sin(q[1] + q[2])*p[0] - a[1]*torch.cos(q[1] + q[2])*p[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + torch.sin(q[0] + q[1] + q[2])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - torch.sin(q[0] + q[1] + q[2])*p[0] - torch.cos(q[0] + q[1] + q[2])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) + torch.sin(q[0] + q[1] + q[2])*p[1] - torch.cos(q[0] + q[1] + q[2])*p[0] + y[0])**2),
    (-a[0]*torch.sin(q[0] + q[1] + q[2])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2])*p[1] - a[1]*torch.sin(q[1] + q[2])*p[0] - a[1]*torch.cos(q[1] + q[2])*p[1] - a[2]*torch.sin(q[2])*p[0] - a[2]*torch.cos(q[2])*p[1] + torch.sin(q[0] + q[1] + q[2])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - torch.sin(q[0] + q[1] + q[2])*p[0] - torch.cos(q[0] + q[1] + q[2])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) + torch.sin(q[0] + q[1] + q[2])*p[1] - torch.cos(q[0] + q[1] + q[2])*p[0] + y[0])**2)])
    return torch.hstack((res, torch.tensor([0, 0, 0, 0], device=q.device)))


# Link 4
@torch.jit.script
def p4(q, p, a):
    res = torch.hstack([a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) + a[3]*torch.cos(q[0] + q[1] + q[2]) - torch.sin(q[0] + q[1] + q[2] + q[3])*p[1] + torch.cos(q[0] + q[1] + q[2] + q[3])*p[0],
    a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + a[3]*torch.sin(q[0] + q[1] + q[2]) + torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3])*p[1],
    p[2]])
    return res


@torch.jit.script
def d4(q, y, p, a):
    res = torch.sqrt((p[2] - y[2])**2 + (a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + a[3]*torch.sin(q[0] + q[1] + q[2]) + torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] - y[1])**2 + (a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) + a[3]*torch.cos(q[0] + q[1] + q[2]) - torch.sin(q[0] + q[1] + q[2] + q[3])*p[1] + torch.cos(q[0] + q[1] + q[2] + q[3])*p[0] - y[0])**2)
    return res


@torch.jit.script
def r4(q, y, p, a):
    res = torch.hstack([(-a[0]*a[1]*torch.sin(q[0]) - a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] + a[1]*torch.sin(q[0])*y[0] - a[1]*torch.cos(q[0])*y[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) + torch.sin(q[0] + q[1] + q[2] + q[3])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[0] + y[0])**2),
    (-a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] - a[1]*a[2]*torch.sin(q[1]) - a[1]*a[3]*torch.sin(q[1] + q[2]) - a[1]*torch.sin(q[1] + q[2] + q[3])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3])*p[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) + torch.sin(q[0] + q[1] + q[2] + q[3])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[0] + y[0])**2),
    (-a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] - a[1]*a[3]*torch.sin(q[1] + q[2]) - a[1]*torch.sin(q[1] + q[2] + q[3])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3])*p[1] - a[2]*a[3]*torch.sin(q[2]) - a[2]*torch.sin(q[2] + q[3])*p[0] - a[2]*torch.cos(q[2] + q[3])*p[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) + torch.sin(q[0] + q[1] + q[2] + q[3])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[0] + y[0])**2),
    (-a[0]*torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] - a[1]*torch.sin(q[1] + q[2] + q[3])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3])*p[1] - a[2]*torch.sin(q[2] + q[3])*p[0] - a[2]*torch.cos(q[2] + q[3])*p[1] - a[3]*torch.sin(q[3])*p[0] - a[3]*torch.cos(q[3])*p[1] + torch.sin(q[0] + q[1] + q[2] + q[3])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - torch.sin(q[0] + q[1] + q[2] + q[3])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) + torch.sin(q[0] + q[1] + q[2] + q[3])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3])*p[0] + y[0])**2)])
    return torch.hstack((res, torch.tensor([0, 0, 0], device=q.device)))

#
# Link 5
@torch.jit.script
def p5(q, p, a):
    res = torch.hstack([a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) + a[3]*torch.cos(q[0] + q[1] + q[2]) + a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0],
    a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1],
    p[2]])
    return res


@torch.jit.script
def d5(q, y, p, a):
    res = torch.sqrt((p[2] - y[2])**2 + (a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - y[1])**2 + (a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) + a[3]*torch.cos(q[0] + q[1] + q[2]) + a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - y[0])**2)
    return res


@torch.jit.script
def r5(q, y, p, a):
    res = torch.hstack([(-a[0]*a[1]*torch.sin(q[0]) - a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + a[1]*torch.sin(q[0])*y[0] - a[1]*torch.cos(q[0])*y[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3])*y[0] - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + y[0])**2),
    (-a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - a[1]*a[2]*torch.sin(q[1]) - a[1]*a[3]*torch.sin(q[1] + q[2]) - a[1]*a[4]*torch.sin(q[1] + q[2] + q[3]) - a[1]*torch.sin(q[1] + q[2] + q[3] + q[4])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3] + q[4])*p[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3])*y[0] - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + y[0])**2),
    ((a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + y[0]) + (-a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + y[0])**2),
    ((a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + y[0]) + (-a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + y[0])**2),
    (-a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - a[1]*torch.sin(q[1] + q[2] + q[3] + q[4])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3] + q[4])*p[1] - a[2]*torch.sin(q[2] + q[3] + q[4])*p[0] - a[2]*torch.cos(q[2] + q[3] + q[4])*p[1] - a[3]*torch.sin(q[3] + q[4])*p[0] - a[3]*torch.cos(q[3] + q[4])*p[1] - a[4]*torch.sin(q[4])*p[0] - a[4]*torch.cos(q[4])*p[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*p[0] + y[0])**2)])
    return torch.hstack((res, torch.tensor([0, 0], device=q.device)))


# Link 6
@torch.jit.script
def p6(q, p, a):
    res = torch.hstack([a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) + a[3]*torch.cos(q[0] + q[1] + q[2]) + a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0],
    a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1],
    p[2]])
    return(res)


@torch.jit.script
def d6(q, y, p, a):
    res = torch.sqrt((p[2] - y[2])**2 + (a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - y[1])**2 + (a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) + a[3]*torch.cos(q[0] + q[1] + q[2]) + a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - y[0])**2)
    return res


@torch.jit.script
def r6(q, y, p, a):
    res = torch.hstack([(-a[0]*a[1]*torch.sin(q[0]) - a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[0]*a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + a[1]*torch.sin(q[0])*y[0] - a[1]*torch.cos(q[0])*y[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3])*y[0] - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3])*y[1] + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*y[0] - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0])**2),
    (-a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[0]*a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - a[1]*a[2]*torch.sin(q[1]) - a[1]*a[3]*torch.sin(q[1] + q[2]) - a[1]*a[4]*torch.sin(q[1] + q[2] + q[3]) - a[1]*a[5]*torch.sin(q[1] + q[2] + q[3] + q[4]) - a[1]*torch.sin(q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3])*y[0] - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3])*y[1] + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*y[0] - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0])**2),
    ((a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0]) + (-a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0])**2),
    ((a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0]) + (-a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0])**2),
    ((a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0]) + (-a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0])**2),
    (-a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - a[1]*torch.sin(q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - a[2]*torch.sin(q[2] + q[3] + q[4] + q[5])*p[0] - a[2]*torch.cos(q[2] + q[3] + q[4] + q[5])*p[1] - a[3]*torch.sin(q[3] + q[4] + q[5])*p[0] - a[3]*torch.cos(q[3] + q[4] + q[5])*p[1] - a[4]*torch.sin(q[4] + q[5])*p[0] - a[4]*torch.cos(q[4] + q[5])*p[1] - a[5]*torch.sin(q[5])*p[0] - a[5]*torch.cos(q[5])*p[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*p[0] + y[0])**2)])
    return torch.hstack((res, torch.tensor(0, device=q.device)))


# # Link 7
@torch.jit.script
def p7(q, p, a):
    res = torch.hstack([a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) + a[3]*torch.cos(q[0] + q[1] + q[2]) + a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0],
    a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1],
    p[2]])
    return res


@torch.jit.script
def d7(q, y, p, a):
    res = torch.sqrt((p[2] - y[2])**2 + (a[1]*torch.sin(q[0]) + a[2]*torch.sin(q[0] + q[1]) + a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - y[1])**2 + (a[0] + a[1]*torch.cos(q[0]) + a[2]*torch.cos(q[0] + q[1]) + a[3]*torch.cos(q[0] + q[1] + q[2]) + a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) + a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - y[0])**2)
    return res


@torch.jit.script
def r7(q, y, p, a):
    res = torch.hstack([(-a[0]*a[1]*torch.sin(q[0]) - a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[0]*a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[0]*a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + a[1]*torch.sin(q[0])*y[0] - a[1]*torch.cos(q[0])*y[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3])*y[0] - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3])*y[1] + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*y[0] - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*y[1] + a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*y[0] - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0])**2),
    (-a[0]*a[2]*torch.sin(q[0] + q[1]) - a[0]*a[3]*torch.sin(q[0] + q[1] + q[2]) - a[0]*a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[0]*a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[0]*a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - a[1]*a[2]*torch.sin(q[1]) - a[1]*a[3]*torch.sin(q[1] + q[2]) - a[1]*a[4]*torch.sin(q[1] + q[2] + q[3]) - a[1]*a[5]*torch.sin(q[1] + q[2] + q[3] + q[4]) - a[1]*a[6]*torch.sin(q[1] + q[2] + q[3] + q[4] + q[5]) - a[1]*torch.sin(q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + a[2]*torch.sin(q[0] + q[1])*y[0] - a[2]*torch.cos(q[0] + q[1])*y[1] + a[3]*torch.sin(q[0] + q[1] + q[2])*y[0] - a[3]*torch.cos(q[0] + q[1] + q[2])*y[1] + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3])*y[0] - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3])*y[1] + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4])*y[0] - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4])*y[1] + a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*y[0] - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5])*y[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0])**2),
    ((a[3]*torch.sin(q[0] + q[1] + q[2]) + a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0]) + (-a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0])**2),
    ((a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) + a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0]) + (-a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0])**2),
    ((a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) + a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0]) + (-a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0])**2),
    ((a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1])*(-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0]) + (-a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0])*(-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1]))/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0])**2),
    (-a[0]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - a[0]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - a[1]*torch.sin(q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - a[1]*torch.cos(q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - a[2]*torch.sin(q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - a[2]*torch.cos(q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - a[3]*torch.sin(q[3] + q[4] + q[5] + q[6])*p[0] - a[3]*torch.cos(q[3] + q[4] + q[5] + q[6])*p[1] - a[4]*torch.sin(q[4] + q[5] + q[6])*p[0] - a[4]*torch.cos(q[4] + q[5] + q[6])*p[1] - a[5]*torch.sin(q[5] + q[6])*p[0] - a[5]*torch.cos(q[5] + q[6])*p[1] - a[6]*torch.sin(q[6])*p[0] - a[6]*torch.cos(q[6])*p[1] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0]*y[0] + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1]*y[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0]*y[1] + torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1]*y[0])/torch.sqrt((p[2] - y[2])**2 + (-a[1]*torch.sin(q[0]) - a[2]*torch.sin(q[0] + q[1]) - a[3]*torch.sin(q[0] + q[1] + q[2]) - a[4]*torch.sin(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) - torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] + y[1])**2 + (-a[0] - a[1]*torch.cos(q[0]) - a[2]*torch.cos(q[0] + q[1]) - a[3]*torch.cos(q[0] + q[1] + q[2]) - a[4]*torch.cos(q[0] + q[1] + q[2] + q[3]) - a[5]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4]) - a[6]*torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5]) + torch.sin(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[1] - torch.cos(q[0] + q[1] + q[2] + q[3] + q[4] + q[5] + q[6])*p[0] + y[0])**2)])
    return res


p_arr = [p1, p2, p3, p4, p5, p6, p7]
d_arr = [d1, d2, d3, d4, d5, d6, d7]
r_arr = [r1, r2, r3, r4, r5, r6, r7]

def lambda_pos(q, link_id: int, p, a):
    return p_arr[link_id](q, p, a)
    # if link_id == 0:
    #     return p1(q, p, a)
    # elif link_id == 1:
    #     return p2(q, p, a)
    # elif link_id == 2:
    #     return p3(q, p, a)
    # elif link_id == 3:
    #     return p4(q, p, a)
    # elif link_id == 4:
    #     return p5(q, p, a)
    # elif link_id == 5:
    #     return p6(q, p, a)
    # elif link_id == 6:
    #     return p7(q, p, a)
    # else:
    #     return p1(q, p, a)*0

    # f_dict = {0 : p1, 1 : p2, 2 : p3, 3 : p4, 4 : p5, 5 : p6, 6 : p7}
    # return f_dict[link_id](q, p, a)
    #return locals()['p{}'.format(link_id+1)](q, p, a)
    # p_1 = p1(q, p, a)
    # p_2 = p2(q, p, a)
    # p_3 = p3(q, p, a)
    # p_4 = p4(q, p, a)
    # p_5 = p5(q, p, a)
    # p_6 = p6(q, p, a)
    # p_7 = p7(q, p, a)
    # arr = [p_1, p_2, p_3, p_4, p_5, p_6, p_7]
    # return(arr[link_id])


def lambda_dist(q, y, link_id: int, p, a):
    return d_arr[link_id](q, y, p, a)
    # if link_id == 0:
    #     return d1(q, y, p, a)
    # elif link_id == 1:
    #     return d2(q, y, p, a)
    # elif link_id == 2:
    #     return d3(q, y, p, a)
    # elif link_id == 3:
    #     return d4(q, y, p, a)
    # elif link_id == 4:
    #     return d5(q, y, p, a)
    # elif link_id == 5:
    #     return d6(q, y, p, a)
    # elif link_id == 6:
    #     return d7(q, y, p, a)
    # else:
    #     return d1(q, y, p, a)*0


#@torch.jit.script
def lambda_rep(q, y, link_id: int, p, a):
    # it's impossible to jit-script the function handle, so we lose the jit starting here
    # attempt to use a switch to store the function handles allows to jit it, but it is very slow still
    return r_arr[link_id](q, y, p, a)
    # if link_id == 0:
    #     return r1(q, y, p, a)
    # elif link_id == 1:
    #     return r2(q, y, p, a)
    # elif link_id == 2:
    #     return r3(q, y, p, a)
    # elif link_id == 3:
    #     return r4(q, y, p, a)
    # elif link_id == 4:
    #     return r5(q, y, p, a)
    # elif link_id == 5:
    #     return r6(q, y, p, a)
    # elif link_id == 6:
    #     return r7(q, y, p, a)
    # else:
    #     return r1(q, y, p, a)*0

def lambda_dist_vec(q, y, link_id, p, a):
    res = torch.zeros_like(q)
    for i in range(q.shape[0]):
        res[i] = lambda_dist(q[i], y, link_id[i], p, a)
    return res

def lambda_pos_vec(q, link_id, p, a):
    res = torch.zeros_like(q)
    for i in range(q.shape[0]):
        res[i] = lambda_pos(q[i], link_id[i], p, a)
    return res


#@torch.jit.script
def lambda_rep_vec(q, y, link_id, p, a):
    res = torch.zeros([q.shape[0], 7]).to(q.device)
    for i in range(q.shape[0]):
        res[i] = lambda_rep(q[i], y[i], link_id[i], p[i], a)
    return res


def main():
    params = {'device': 'cuda:0', 'dtype': torch.float32}
    q = torch.tensor([0, 0, 0, 0, 0, 0, 0]).to(**params)
    p = torch.tensor([0, 0, 0]).to(**params)
    a = torch.tensor([0, 1, 1, 1, 1, 1, 1, 1]).to(**params)
    y = torch.tensor([2, 2, 0]).to(**params)
    q = torch.tile(q, (100, 1))
    links = torch.randint(7, (100, 1)).squeeze()
    for i in range(100):
        if i == 10:
            t0 = time.time()
        rep = lambda_rep_vec(q, y, links, p, a)
    tf = time.time()
    print(tf-t0)
    #print(pos, dst, rep)


if __name__ == '__main__':
    main()
