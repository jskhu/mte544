import numpy as np


class ANode:
    def __init__(self, parent, node):
        self.parent = parent
        self.node = node

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.node == other.node

    def __str__(self):
        return "k: {} g: {} h: {} f: {}".format(self.node, self.g, self.h, self.f)

    def __repr__(self):
        return str(self)


def a_star(nodes, connections, start, end):
    start_node = ANode(None, start)
    end_node = ANode(None, end)
    print(start_node)
    print(end_node)

    open_list = []
    closed_list = []

    open_list.append(start_node)

    while len(open_list) > 0:
        current_index, current_node = min(
            enumerate(open_list), key=lambda item: item[1].f)

        open_list.pop(current_index)
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.node)
                current = current.parent
            return path[::-1]

        children = []
        for child_index in range(len(connections)):
            if connections[current_node.node.id][child_index] and current_node.node.id != child_index:
                children.append(ANode(current_node, nodes[child_index]))

        for child in children:
            if child in closed_list:
                continue

            child.g = current_node.g + \
                np.linalg.norm(current_node.node.val - child.node.val)
            child.h = np.linalg.norm(child.node.val - end_node.node.val)
            child.f = child.g + child.h

            add_to_open_list = True
            for open_node in open_list:
                if child == open_node and child.g >= open_node.g:
                    add_to_open_list = False
                    break

            if add_to_open_list:
                open_list.append(child)
