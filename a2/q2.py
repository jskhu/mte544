import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines

from helpers import read_pgm, simulate
from objects import Circle, Line, Robot
from map import get_map
from kd_tree import KdTree
from a_star import a_star

if __name__ == "__main__":
    pgm = read_pgm('sim_map.pgm')

    # simulation code
    t_max = 25
    dt = 0.1
    ts = np.arange(0, t_max + dt, dt)
    length = 10
    num_pix = 100
    m = get_map(pgm)

    # Robot parameters
    b = 4.5
    robot = Robot(np.array([5, 5, np.radians(90)]), b)

    # Generate milestones for kd-tree
    np.random.seed(42)
    coordinates = np.random.rand(70, 2) * 97 + 1

    milestones = []
    milestones.append(np.array([5,5]))

    for coord in coordinates:
        circle = Circle(coord, robot.b/2)
        if len(m.get_intersecting_rects(circle)) == 0:
            milestones.append(coord)
    milestones.append(np.array([70, 15]))
    milestones.append(np.array([90, 50]))
    milestones.append(np.array([30, 95]))
    milestones.append(np.array([5, 50]))

    milestones = np.array(milestones)

    # Generate kd-tree
    tree = KdTree(milestones)
    tree.init_build()

    # Create connection/adjacency matrix for nodes
    # Also remove invalid connections
    connections = np.zeros((tree.nodes.shape[0], tree.nodes.shape[0]), dtype=int)
    for milestone in milestones:
        best = tree.get_nn(milestone, 5)
        
        for b in best:
            line_l = Line(best[0].val - robot.b/2, b.val - robot.b/2)
            line_r = Line(best[0].val + robot.b/2, b.val + robot.b/2)

            if m.intersects_line(line_l) or m.intersects_line(line_r):
                continue
            
            connections[best[0].id,b.id] = 1
            connections[b.id, best[0].id] = 1

    fig, ax = plt.subplots()
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')

    ax.scatter(milestones[:,0], milestones[:,1])

    # only print out lower triangle matrix values since connections is symmetric
    for j in range(len(connections)):
        for i in range(j+1):
            if connections[j][i]:
                a = tree.nodes[j]
                b = tree.nodes[i]
                line_plt = lines.Line2D([a.val[0], b.val[0]], [a.val[1], b.val[1]])
                ax.add_line(line_plt)

    # Path 1
    path = a_star(tree.nodes, connections, tree.nodes[0], tree.nodes[-4])
    values, headings = simulate(robot, m, path, ts)
    fig, ax = plt.subplots()
    ax.plot(values[:,0], values[:,1], color='g')

    ax.quiver(values[::10,0], values[::10,1], headings[::10,0], -headings[::10,1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')

    ax.scatter(milestones[:,0], milestones[:,1])
    for i in range(len(path)-1):
        line_plt = lines.Line2D([path[i].val[0], path[i+1].val[0]], [path[i].val[1], path[i+1].val[1]], color='r')
        ax.add_line(line_plt)

    # Path 2
    path = a_star(tree.nodes, connections, tree.nodes[-4], tree.nodes[-3])
    values, headings = simulate(robot, m, path, ts)
    fig, ax = plt.subplots()
    ax.plot(values[:,0], values[:,1], color='g')

    ax.quiver(values[::10,0], values[::10,1], headings[::10,0], -headings[::10,1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')

    ax.scatter(milestones[:,0], milestones[:,1])
    for i in range(len(path)-1):
        line_plt = lines.Line2D([path[i].val[0], path[i+1].val[0]], [path[i].val[1], path[i+1].val[1]], color='r')
        ax.add_line(line_plt)

    # Path 3
    path = a_star(tree.nodes, connections, tree.nodes[-3], tree.nodes[-2])
    values, headings = simulate(robot, m, path, ts)
    fig, ax = plt.subplots()
    ax.plot(values[:,0], values[:,1], color='g')

    ax.quiver(values[::10,0], values[::10,1], headings[::10,0], -headings[::10,1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')

    ax.scatter(milestones[:,0], milestones[:,1])
    for i in range(len(path)-1):
        line_plt = lines.Line2D([path[i].val[0], path[i+1].val[0]], [path[i].val[1], path[i+1].val[1]], color='r')
        ax.add_line(line_plt)

    # Path 4
    path = a_star(tree.nodes, connections, tree.nodes[-2], tree.nodes[-1])
    values, headings = simulate(robot, m, path, ts)
    fig, ax = plt.subplots()
    ax.plot(values[:,0], values[:,1], color='g')

    ax.quiver(values[::10,0], values[::10,1], headings[::10,0], -headings[::10,1])
    ax.imshow(pgm)
    ax.set_xlabel('Position (pixels)')
    ax.set_ylabel('Position (pixels)')

    ax.scatter(milestones[:,0], milestones[:,1])
    for i in range(len(path)-1):
        line_plt = lines.Line2D([path[i].val[0], path[i+1].val[0]], [path[i].val[1], path[i+1].val[1]], color='r')
        ax.add_line(line_plt)
    
    plt.show()
