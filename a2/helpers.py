import numpy as np
from objects import Circle


def read_pgm(pgmf):
    with open(pgmf, 'rb') as f:
        """Return a raster of integers from a PGM as a list of lists."""
        header = f.readline()
        print(header[0], header[1])
        assert header == b'P5\n'
        while True:
            l = f.readline()
            if not l[0] == 35:   # skip any header comment lines
                break
        (width, height) = [int(i) for i in l.split()]
        depth = int(f.readline())
        assert depth <= 255

        raster = []
        for y in range(height):
            row = []
            for y in range(width):
                row.append(ord(f.read(1)))
            raster.append(row)

    return np.array(raster)


def simulate(robot, m, path, ts):
    grad_max = 10
    grad_min = -10
    dt = 0.1
    path_index = 1
    values = []
    headings = []
    values.append(robot.get_pose())
    headings.append(robot.get_unit_heading())
    for step, t in enumerate(ts):
        # calculate gradient
        # Add attraction
        grad = (path[path_index].val - robot.pose[:-1])
        if np.linalg.norm(grad) < 3:
            path_index += 1
            if path_index == len(path):
                break
            continue

        grad[0] = max(grad_min, min(grad[0], grad_max))
        grad[1] = max(grad_min, min(grad[1], grad_max))

        unit_grad = grad / np.linalg.norm(grad)
        unit_robot = robot.get_unit_heading()

        side = unit_grad[1] * unit_robot[0] - unit_grad[0] * unit_robot[1]
        angle = np.sign(side) * np.arccos(np.dot(unit_grad, unit_robot))

        v = np.sqrt(grad[0]**2 + grad[1]**2)
        if angle > np.radians(40):
            v = 0
        w = max(-3/2*np.pi, min(3/2*np.pi, angle))

        new_robot_pose = robot.get_update(v, w, dt)
        robot_circle = Circle(new_robot_pose[:-1], robot.b / 2)
        intersects = m.get_intersecting_rects(robot_circle)

        if len(intersects) > 0:
            v = 0
            w = np.sign(w) * np.pi/2
        robot.update(v, w, dt)
        values.append(robot.get_pose())
        headings.append(robot.get_unit_heading())
    return np.array(values), np.array(headings)
