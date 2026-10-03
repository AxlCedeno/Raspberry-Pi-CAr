def regular_polygon(n, radius=1, center=(0, 0)):
    cx, cy = center
    angles = num.linspace(0, 2*num.pi, n+1, endpoint=True)

    return [
        (cx + radius*num.cos(a), cy + radius*num.sin(a))
        for a in angles
    ]

def ring(cx, cy, oradius, iradius):
    return Wedge(center = (cx,cy),r = oradius, theta1 = 0, theta2 = 360, width = oradius-iradius, facecolor = 'blue', edgecolor = 'black')

def square(c1x, c1y, c2x, c2y, c3x, c3y, c4x, c4y, fc, ec):
    square = ((c1x, c1y),
           (c2x, c2y),
           (c3x, c3y),
           (c4x, c4y),
           (c1x, c1y))
    square_path = Path(square)
    return patches.PathPatch(square_path, facecolor = fc, edgecolor = ec)

def testerx(xi):  
    x = round(random.uniform(-9.0, 9.0),3)
    return x
def testery(yi):
    y = round(random.uniform(-9.0, 9.0),3)
    return y

def euclidean(x, y, coordinates):
    distance2 = 100
    for coordinate in coordinates:
        point1 = num.array([x, y])
        point2 = num.array(coordinate)

        distance = num.linalg.norm(point1-point2)
        
        if distance < distance2:
            distance2 = distance
            current = coordinate

    return current

def distance(p1, p2):
    p1 = num.array(p1)
    p2 = num.array(p2)
    return num.linalg.norm(p1 - p2)

def line_hits_obstacle(p1, p2, paths, samples=20):
    xs = num.linspace(p1[0], p2[0], samples)
    ys = num.linspace(p1[1], p2[1], samples)

    for x, y in zip(xs, ys):
        for path in paths:
            if path.contains_point((x, y), radius=0.05):
                return True
    return False

def near_nodes(p_new, coordinates, radius):
    neighbors = []
    for node in coordinates:
        if distance(node, p_new) <= radius:
            neighbors.append(tuple(node))
    return neighbors 

def steer(pEucl, pRand, epsilon):
    pEucl = num.array(pEucl)
    pRand = num.array(pRand)

    direction = pRand - pEucl
    dist = num.linalg.norm(direction)

    if dist == 0:
        return p_near.tolist()

    direction = direction / dist   # normalize

    p_new = pEucl + epsilon * direction

    return p_new.tolist()
