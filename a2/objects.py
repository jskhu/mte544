import numpy as np
import matplotlib.pyplot as plt


class Robot:
    def __init__(self, pose, b):
        self.pose = pose
        self.b = b

    def get_update(self, v, w, dt):
        pose = np.copy(self.pose)
        pose[0] += v * np.cos(pose[2]) * dt
        pose[1] += v * np.sin(pose[2]) * dt
        pose[2] += w * dt
        return pose

    def update(self, v, w, dt):
        self.pose = self.get_update(v, w, dt)

    def get_pose(self):
        return np.copy(self.pose)

    def get_unit_heading(self):
        robot_dist = np.linalg.norm(self.pose[:-1])
        robot_heading = np.array(
            [robot_dist * np.cos(self.pose[2]), robot_dist * np.sin(self.pose[2])])
        unit_robot = robot_heading / np.linalg.norm(robot_heading)
        return unit_robot


class Circle:
    def __init__(self, p, r):
        self.p = p
        self.r = r

    def get_plt(self):
        return plt.Circle(self.p, self.r, color='r')


class Line:
    def __init__(self, p0, p1):
        self.p0 = p0
        self.p1 = p1

    def orientation(self, p, q, r):
        val = ((q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]))

        if val > 0:
            return 1
        elif val < 0:
            return 2
        return 0

    def intersects_line(self, line):
        o1 = self.orientation(self.p0, self.p1, line.p0)
        o2 = self.orientation(self.p0, self.p1, line.p1)
        o3 = self.orientation(line.p0, line.p1, self.p0)
        o4 = self.orientation(line.p0, line.p1, self.p1)

        if (o1 != o2) and (o3 != o4):
            return True

        return False

    def intersects_circle(self, circle):
        d = self.p1 - self.p0
        f = self.p0 - circle.p

        a = d.dot(d)
        b = 2*f.dot(d)
        c = f.dot(f) - circle.r**2

        discriminant = b*b - 4*a*c

        if discriminant < 0:
            return False

        discriminant = np.sqrt(discriminant)

        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)

        if t1 >= 0 and t1 <= 1:
            return True

        if t2 >= 0 and t2 <= 1:
            return True

        return False


class Rectangle:
    def __init__(self, a, d):
        self.a = a
        self.b = np.array([d[0], a[1]])
        self.c = np.array([a[0], d[1]])
        self.d = d

        self.lines = np.array([
            Line(self.a, self.b),
            Line(self.a, self.c),
            Line(self.b, self.d),
            Line(self.c, self.d)
        ])

    def get_centroid(self):
        return (self.a + self.b + self.c + self.d) * 1.0 / 4

    def get_corners(self):
        return np.array([
            self.a,
            self.b,
            self.c,
            self.d
        ])

    def intersects_line(self, line):
        intersects = False
        for l in self.lines:
            intersects |= l.intersects_line(line)

        return intersects

    def intersects_circle(self, circle):
        intersects = False
        for line in self.lines:
            intersects |= line.intersects_circle(circle)

        intersects |= self.is_point_inside(circle.p)
        intersects |= self.is_inside_circle(circle)
        return intersects

    def is_point_inside(self, p):
        ap = p - self.a
        ab = self.b - self.a
        ac = self.c - self.a

        return 0 <= ap.dot(ab) \
            and ap.dot(ab) <= ab.dot(ab) \
            and 0 <= ap.dot(ac) \
            and ap.dot(ac) <= ac.dot(ac)

    def is_inside_circle(self, circle):
        ra = np.linalg.norm(self.a - circle.p)
        rb = np.linalg.norm(self.b - circle.p)
        rc = np.linalg.norm(self.c - circle.p)
        rd = np.linalg.norm(self.d - circle.p)
        return ra < circle.r \
            and rb < circle.r \
            and rc < circle.r \
            and rd < circle.r
