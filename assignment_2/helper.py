from math import atan2, pi, cos, sin


def computeLineThroughTwoPoints(p1, p2):
    x1 = p1[0]
    x2 = p2[0]
    y1 = p1[1]
    y2 = p2[1]
    a = y2-y1
    b = x1-x2
    c = -(a*x1+b*y1)
    s = (a**2+b**2)**0.5
    return a/s, b/s, c/s


def computeDistancePointToLine(q, p1, p2):
    [a, b, c] = computeLineThroughTwoPoints(p1, p2)
    Distance = abs(a*q[0] + b*q[1] + c)
    return Distance


def DistancePointToPoint(p1, p2):
    Distance = round((((p1[1]-p2[1])**2)+((p1[0]-p2[0])**2))**0.5, 4)
    return Distance


def computeDistancePointToSegment(q, p1, p2):
    [a, b, c] = computeLineThroughTwoPoints(p1, p2)
    x1 = round((b**2)*q[0]-a*b*q[1]-a*c, 4)
    if b == 0:
        y1 = q[1]
    else:
        y1 = -(1/b)*(a*x1+c)

    d1 = DistancePointToPoint([x1, y1], p1)  # perpendicular point to point 1
    d2 = DistancePointToPoint([x1, y1], p2)  # perpendicular point to point 2
    d3 = DistancePointToPoint(p1, p2)  # distance between p1 and p2

    if d1+d2 > d3+0.0001:
        if d1 > d2:
            w = 2
            dist = DistancePointToPoint(q, p2)
            T = p2
        else:
            w = 1
            dist = DistancePointToPoint(q, p1)
            T = p1
    else:
        w = 0
        dist = computeDistancePointToLine(q, p1, p2)
        T = [x1, y1]
    return dist, w, T


def computeDistancePointToPolygon(q, P):
    l = len(P)
    d = []
    d.append(computeDistancePointToSegment(q, P[l-1], P[0]))
    for i in range(l-1):
        d.append(computeDistancePointToSegment(q, P[i], P[i+1]))
    return (min(d))


def computeTangentVectorToPolygon(q, P):
    [dis, w, T] = computeDistancePointToPolygon(q, P)
    theta = atan2(q[0]-T[0], T[1]-q[1])
    Vx = round(cos(theta), 4)
    Vy = round(sin(theta), 4)
    if theta < 0:
        theta += 2*pi
    return Vx, Vy, theta


def VectorToPoint(q, T):
    theta = atan2(T[1]-q[1], T[0]-q[0])
    Vx = round(cos(theta), 4)
    Vy = round(sin(theta), 4)
    if theta < 0:
        theta += 2*pi
    return Vx, Vy, theta


def attPotential(q, g):
    D = DistancePointToPoint(q, g)
    if D > 2:
        Vx = 1.6*(g[0]-q[0])/D
        Vy = 1.6*(g[1]-q[1])/D
    else:
        Vx = 0.8*(g[0]-q[0])
        Vy = 0.8*(g[1]-q[1])
    return Vx, Vy


def repPotential(q, P):
    [D, w, [a, b]] = computeDistancePointToPolygon(q, P)
    if D > 2:
        Vx = 0
        Vy = 0
    else:
        Vx = 0.8*(0.5-(1/D))*(a-q[0])/(D**3)
        Vy = 0.8*(0.5-(1/D))*(b-q[1])/(D**3)
    return Vx, Vy


def normalise(q):
    s = (q[0]**2)+(q[1]**2)
    if s == 0:
        Vx = 0
        Vy = 0
    else:
        Vx = q[0]/(s**0.5)
        Vy = q[1]/(s**0.5)
    theta = atan2(Vy, Vx)
    return Vx, Vy, theta
