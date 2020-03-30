import numpy as np
import matplotlib.pyplot as plt

from helpers import read_pgm
from objects import Circle, Robot
from map import get_map

if __name__ == "__main__":
    pgm = read_pgm('sim_map.pgm')

    # simulation code
    t_max = 30
    dt = 0.1
    ts = np.arange(0, t_max + dt, dt)
    m = get_map(pgm)

    # Robot parameters
    b = 4.5
    end_pos = np.array([95.5, 95.5])
    robot = Robot(np.array([5.5, 5.5, np.radians(90)]), b)

    # Potential Field params
    rep_radius = 4.5
    k_att = 0.2
    k_rep = 5000
    rho0 = 4.5
    grad_max = 10
    grad_min = -10

    values = []
    values.append(robot.get_pose())
    for step, t in enumerate(ts):
        # calculate gradient
        # Add attraction
        grad = -k_att * (robot.pose[:-1] - end_pos)

        # Add repulsion
        # Radius check around for intersections
        rep_circle = Circle(robot.pose[:-1], rep_radius)
        nearby_rects = m.get_intersecting_rects(rep_circle)

        for i in nearby_rects:
            rect = m.get_rect(i)
            dist = np.linalg.norm(robot.pose[:-1] - rect.get_centroid())
            grad += -k_rep * (1 / dist - 1 / rho0) * \
                (robot.pose[:-1] - rect.get_centroid()) / (dist**3)

        grad[0] = max(grad_min, min(grad[0], grad_max))
        grad[1] = max(grad_min, min(grad[1], grad_max))

        unit_grad = grad / np.linalg.norm(grad)

        robot_dist = np.linalg.norm(robot.pose[:-1])
        robot_heading = np.array(
            [robot_dist * np.cos(robot.pose[2]), robot_dist * np.sin(robot.pose[2])])
        unit_robot = robot_heading / np.linalg.norm(robot_heading)

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
            w = -np.sign(w) * np.pi/2

        robot.update(v, w, dt)
        values.append(robot.get_pose())

    values = np.array(values)

    fig, ax = plt.subplots()
    ax.plot(values[:, 0], values[:, 1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')
    circle1 = plt.Circle(np.array([5, 5]), 4.5 / 2, color='r')
    circle2 = plt.Circle(np.array([95, 95]), 4.5 / 2, color='b')

    ax.add_artist(circle1)
    ax.add_artist(circle2)
    plt.show()
