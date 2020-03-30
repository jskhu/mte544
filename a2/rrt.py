import numpy as np

from objects import Circle, Line
from kd_tree import Node

class RRT:
    def __init__(self, robot, m, step_size=10):
        self.m = m
        self.robot = robot
        self.nodes = []
        self.edges = []
        self.step_size = step_size

    def build(self, start_pos, end_pos, n):
        q_start = Node(start_pos, 0)
        q_end = Node(end_pos, 1)
        self.nodes.append(q_start)
        self.nodes.append(q_end)
        for i in range(n):
            # create random node
            choice = np.random.choice(2, p=[0.9, 0.1])
            q_rand = Node(np.random.rand(2,) * 97 + 1, None)
            if choice:
                q_rand = q_end
            q_near = self.find_closest_node(q_rand, q_end)
            u_rand = q_rand.val - q_near.val
            u_rand = u_rand / np.linalg.norm(u_rand)
            q_new = Node(q_near.val + u_rand * self.step_size, len(self.nodes))

            # Check collision

            if self.collision_detected(q_new, q_near):
                continue
            
            self.nodes.append(q_new)
            self.edges.append(np.array([q_near.id, q_new.id]))

            # Check if close to end_node
            if np.linalg.norm(q_new.val - q_end.val) < self.step_size and not self.collision_detected(q_new, q_end):
                self.edges.append(np.array([q_new.id, q_end.id]))
        
        self.nodes = np.array(self.nodes)
        self.edges = np.array(self.edges)
        
    def find_closest_node(self, q_rand, q_end):
        closest = None
        closest_dist = 999999
        for node in self.nodes:
            if node == q_end:
                continue
            dist = np.linalg.norm(q_rand.val - node.val)
            if dist < closest_dist:
                closest = node
                closest_dist = dist
        
        return closest

    def collision_detected(self, q_new, q_near):
        if np.any(q_new.val < 0 + self.robot.b/2) or np.any(q_new.val > 99 - self.robot.b/2):
                return True

        circle = Circle(q_new.val, self.robot.b)
        line_l = Line(q_near.val - self.robot.b, q_new.val - self.robot.b)
        line_r = Line(q_near.val + self.robot.b, q_new.val + self.robot.b)
        if (len(self.m.get_intersecting_rects(circle)) > 0
            or self.m.intersects_line(line_l)
            or self.m.intersects_line(line_r)):
            return True

        return False
    
    def get_points(self):
        points = []
        for node in self.nodes:
            points.append(node.val)
        return np.array(points) 

    def get_connections(self):
        connections = np.zeros((self.nodes.shape[0], self.nodes.shape[0]), dtype=int)

        for edge in self.edges:
            connections[edge[0], edge[1]] = 1
            connections[edge[1], edge[0]] = 1

        return connections