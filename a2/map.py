import numpy as np

from objects import Rectangle


class Map:
    def __init__(self, pgm, rects):
        self.pgm = pgm
        self.rects = rects

    def get_rect(self, i):
        return self.rects[i]

    def intersects_line(self, line):
        intersects = False
        for rect in self.rects:
            intersects |= rect.intersects_line(line)

        return intersects

    def get_intersecting_rects(self, circle):
        intersecting_rects = set()
        for i, rect in enumerate(self.rects):
            if rect.intersects_circle(circle):
                intersecting_rects.add(i)
        return intersecting_rects

    def is_point_inside(self, point):
        for i, rect in enumerate(self.rects):
            if rect.is_point_inside(point):
                return True
        return False


def get_map(pgm):
    # find corners
    corners = []
    for j in range(1, 99):
        for i in range(1, 99):
            if not pgm[j][i]:
                # Check if corner
                count = 0
                count += 1 if not pgm[j-1][i] else 0
                count += 1 if not pgm[j+1][i] else 0
                count += 1 if not pgm[j][i-1] else 0
                count += 1 if not pgm[j][i+1] else 0
                if (count == 2):
                    corners.append(np.array([i, j]))

    # add 4 additional corners
    corners.append(np.array([corners[5][0], corners[6][1]]))
    corners.append(np.array([corners[5][0], corners[11][1]]))
    corners.append(np.array([corners[5][0], corners[20][1]]))
    corners.append(np.array([corners[5][0], corners[23][1]]))

    corners = np.array(corners)

    rect0 = Rectangle(corners[0], corners[3])
    rect1 = Rectangle(corners[4], corners[27])
    rect2 = Rectangle(corners[28], corners[11])
    rect3 = Rectangle(corners[30], corners[23])
    rect4 = Rectangle(corners[7], corners[13])
    rect5 = Rectangle(corners[9], corners[15])
    rect6 = Rectangle(corners[16], corners[19])
    rect7 = Rectangle(corners[21], corners[25])

    rectangles = [
        rect0,
        rect1,
        rect2,
        rect3,
        rect4,
        rect5,
        rect6,
        rect7
    ]

    m = Map(pgm, rectangles)
    return m
