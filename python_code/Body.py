from Vect import *

class Body:
    def __init__(self):
        self.type = "dynamic"
        self.shape = "none"
        self.pos = Vector(0,0)
        self.angle = 0.0
        self.vel = Vector(0,0)
        self.angVel = 0.0
        self.mass = 1
        self.I = 1
        self.w = 0
        self.h = 0
        self.radius = 0
        self.density = 0.3
        self.alive = True
        self.name = "none"

    def inv_mass(self):
        if self.type == "dynamic":
            return 1/self.mass
        else :
            return 0.0
    def inv_I(self):
        if self.type == "dynamic":
            return 1/self.I
        else :
            return 0.0

class Circle(Body):
    def __init__(self, x, y, r):
        super(Circle, self).__init__()
        self.shape = "circle"
        self.pos = Vector(x, y)
        self.radius = r
        self.area = r * r * 3.14
        self.mass = self.density * self.area /10
        self.I = 0.5 * self.mass * self.radius * self.radius

class Box(Body):
    def __init__(self, x ,y, w, h):
        super(Box, self).__init__()
        self.shape = "box"
        self.pos = Vector(x, y)
        self.w = w
        self.h = h
        self.area = w * h
        self.mass = self.density * self.area
        self.I = self.mass * (self.w*self.w + self.h*self.h) / 12

        self.corners = [
            Vector(w/2, -h/2),
            Vector(-w/2, -h/2),
            Vector(-w/2, h/2),
            Vector(w/2, h/2)
        ]

    def getVertices(self):
        vertices = []

        for i in range(len(self.corners)):
            p1 = self.corners[i]
            vec = Vector(self.pos.x + p1.x, self.pos.y + p1.y)
            vec.rotateRefPoint(self.angle, self.pos)
            vertices.append(vec)

        return vertices

    def getNorms(self):

        vertices = self.getVertices()
        norms = []

        p1 = vertices[0]
        p2 = vertices[1]
        n = Vector(p2.x - p1.x, p2.y - p1.y).normalL() / (self.w)
        norms.append(n)

        p1 = vertices[1]
        p2 = vertices[2]
        n = Vector(p2.x - p1.x, p2.y - p1.y).normalL() / (self.h)
        norms.append(n)

        p1 = vertices[2]
        p2 = vertices[3]
        n = Vector(p2.x - p1.x, p2.y - p1.y).normalL() / (self.w)
        norms.append(n)

        p1 = vertices[3]
        p2 = vertices[0]
        n = Vector(p2.x - p1.x, p2.y - p1.y).normalL() / (self.h)
        norms.append(n)

        return norms






