import math

class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return  Vector(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        return  Vector(self.x / scalar, self.y / scalar)

    def length(self):
        return math.sqrt(self.x*self.x + self.y*self.y)

    def normalize(n):
        rt = n.length()
        return n/rt

    def projectLengthOnto(self, vec2):
        dotProduct = dot(self,vec2)
        len = vec2.length()
        return dotProduct/len

    def normalL(self):
        return Vector(-self.y, self.x)

    def normalR(self):
        return Vector(self.y, -self.x)

    def rotate(self, angle):
        newX = (self.x * math.cos(angle)) - (self.y * math.sin(angle))
        newY = (self.x * math.sin(angle)) + (self.y * math.cos(angle))

        self.x = newX
        self.y = newY

    def rotateRefPoint(self, angle, refP):
        newX = (self.x - refP.x) * math.cos(angle) - (self.y - refP.y) * math.sin(angle) + refP.x
        newY = (self.y - refP.y) * math.cos(angle) + (self.x - refP.x) * math.sin(angle) + refP.y

        self.x = newX
        self.y = newY

    def toVertices(self, vertice):
        return Vector(vertice.x - self.x, vertice.y - self.y)

def dot(vec1, vec2):
    return vec1.x * vec2.x + vec1.y * vec2.y
def cross(vec1, vec2):
    return vec1.x * vec2.y - vec1.y * vec2.x
def scalarXvector (scalar, vec):
    return Vector(-scalar * vec.y, scalar * vec.x)
def vectorXscalar (vec, scalar):
    return Vector(scalar * vec.y, -scalar * vec.x)
def clamp(v, lo, hi):
    if v <= lo:
        return lo
    elif v >= hi:
        return hi
    else:
        return v


