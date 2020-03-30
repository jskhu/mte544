import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines

from helpers import read_pgm, simulate
from objects import Circle, Line, Robot
from map import get_map
from a_star import a_star
from rrt import RRT

if __name__ == "__main__":
    pgm = read_pgm('sim_map.pgm')

    # simulation code
    t_max = 100
    dt = 0.1
    ts = np.arange(0, t_max + dt, dt)
    m = get_map(pgm)
    b = 4.5
    robot = Robot(np.array([5, 5, np.radians(90)]), b)
    np.random.seed(34)

    # Path 1
    rrt = RRT(robot, m)
    rrt.build(robot.pose[:-1], np.array([45, 50]), 500)

    fig, ax = plt.subplots()
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')
    points = rrt.get_points()
    ax.scatter(points[:,0], points[:,1])
    for edge in rrt.edges:
        line_plt = lines.Line2D([points[edge[0]][0], points[edge[1]][0]], [points[edge[0]][1], points[edge[1]][1]])
        ax.add_line(line_plt)

    path = a_star(rrt.nodes, rrt.get_connections(), rrt.nodes[0], rrt.nodes[1])
    values = simulate(robot, m, path, ts)
    fig, ax = plt.subplots()
    ax.plot(values[:,0], values[:,1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')

    ax.scatter(points[:,0], points[:,1])

    for i in range(len(path)-1):
        line_plt = lines.Line2D([path[i].val[0], path[i+1].val[0]], [path[i].val[1], path[i+1].val[1]], color='r')
        ax.add_line(line_plt)

    # Path 2
    rrt = RRT(robot, m)
    rrt.build(robot.pose[:-1], np.array([9, 90]), 500)

    fig, ax = plt.subplots()
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')
    points = rrt.get_points()
    ax.scatter(points[:,0], points[:,1])
    for edge in rrt.edges:
        line_plt = lines.Line2D([points[edge[0]][0], points[edge[1]][0]], [points[edge[0]][1], points[edge[1]][1]])
        ax.add_line(line_plt)

    path = a_star(rrt.nodes, rrt.get_connections(), rrt.nodes[0], rrt.nodes[1])
    values = simulate(robot, m, path, ts)
    fig, ax = plt.subplots()
    ax.plot(values[:,0], values[:,1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')
    ax.scatter(points[:,0], points[:,1])

    for i in range(len(path)-1):
        line_plt = lines.Line2D([path[i].val[0], path[i+1].val[0]], [path[i].val[1], path[i+1].val[1]], color='r')
        ax.add_line(line_plt)

    # Path 3
    rrt = RRT(robot, m)
    rrt.build(robot.pose[:-1], np.array([90, 10]), 500)

    fig, ax = plt.subplots()
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')
    points = rrt.get_points()
    ax.scatter(points[:,0], points[:,1])
    for edge in rrt.edges:
        line_plt = lines.Line2D([points[edge[0]][0], points[edge[1]][0]], [points[edge[0]][1], points[edge[1]][1]])
        ax.add_line(line_plt)

    path = a_star(rrt.nodes, rrt.get_connections(), rrt.nodes[0], rrt.nodes[1])
    values = simulate(robot, m, path, ts)
    fig, ax = plt.subplots()
    ax.plot(values[:,0], values[:,1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')
    ax.scatter(points[:,0], points[:,1])

    for i in range(len(path)-1):
        line_plt = lines.Line2D([path[i].val[0], path[i+1].val[0]], [path[i].val[1], path[i+1].val[1]], color='r')
        ax.add_line(line_plt)

    # Path 4
    rrt = RRT(robot, m)
    rrt.build(robot.pose[:-1], np.array([90, 90]), 500)

    fig, ax = plt.subplots()
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')
    points = rrt.get_points()
    ax.scatter(points[:,0], points[:,1])
    for edge in rrt.edges:
        line_plt = lines.Line2D([points[edge[0]][0], points[edge[1]][0]], [points[edge[0]][1], points[edge[1]][1]])
        ax.add_line(line_plt)

    path = a_star(rrt.nodes, rrt.get_connections(), rrt.nodes[0], rrt.nodes[1])
    values = simulate(robot, m, path, ts)
    fig, ax = plt.subplots()
    ax.plot(values[:,0], values[:,1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')

    ax.scatter(points[:,0], points[:,1])
    for i in range(len(path)-1):
        line_plt = lines.Line2D([path[i].val[0], path[i+1].val[0]], [path[i].val[1], path[i+1].val[1]], color='r')
        ax.add_line(line_plt)
        
    plt.show()