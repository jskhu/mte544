import numpy as np
import sys

class Node:
    def __init__(self, val, id):
        self.val = val
        self.id = id

    def __str__(self):
        return str(self.val)
    
    def __repr__(self):
        return str(self)

    def __eq__(self, other):
        return self.id == other.id

class KNode(Node):
    def __init__(self, val, id):
        super().__init__(val, id)
        self.left = None
        self.right = None

class KdTree:
    def __init__(self, points):
        self.points = points
        self.nodes = []
        for id, point in enumerate(points):
            self.nodes.append(KNode(point, id))
        self.nodes = np.array(self.nodes)

    def init_build(self):
        self.root = self.build(self.points, self.nodes, 0)
    
    def build(self, points, nodes, depth):
        if len(points) == 0:
            return None
        axis = depth % 2

        sorted_indices = np.argsort(points[:, axis])
        sorted_points = points[sorted_indices]
        sorted_nodes = nodes[sorted_indices]
        median_index = len(sorted_nodes) // 2
        node = sorted_nodes[median_index]
        node.left = self.build(sorted_points[:median_index], sorted_nodes[:median_index], depth+1)
        node.right = self.build(sorted_points[median_index+1:], sorted_nodes[median_index+1:], depth+1)

        return node

    def get_nn(self, point, num):
        seen = []

        for i in range(num):
            best = self.nn(self.root, seen, point)
            seen.append(best)

        return seen

    def nn(self, node, seen, point):
        best = KNode(np.array([sys.maxsize, sys.maxsize]), 99999999)
        best_dist = np.linalg.norm(point - best.val)
        if node not in seen:
            best = node
            best_dist = np.linalg.norm(point - best.val)
        if node.left:
            left_best = self.nn(node.left, seen, point)
            left_dist = np.linalg.norm(point - left_best.val)
            if left_dist < best_dist:
                best = left_best
                best_dist = left_dist
        if node.right:
            right_best = self.nn(node.right, seen, point)
            right_dist = np.linalg.norm(point - right_best.val)
            if right_dist < best_dist:
                best = right_best
                best_dist = right_dist

        return best