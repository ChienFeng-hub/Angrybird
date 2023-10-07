from Body import *
import Arbiter
import math

def Collide(Contacts, cir1, cir2):
    distanceSqr = dot((cir1.pos - cir2.pos), (cir1.pos - cir2.pos))
    if distanceSqr > (cir1.radius + cir2.radius) * (cir1.radius + cir2.radius):
        return 0
    else:
        Norm = (cir2.pos - cir1.pos).normalize()
        contactPoint = (cir1.pos + cir2.pos)/2
        penetration = cir1.radius - dot(Norm, contactPoint - cir1.pos)
        contact1 = Arbiter.Contact()
        contact1.normal = Norm
        contact1.separation = -penetration * 2
        contact1.pos = contactPoint
        Contacts.append(contact1)

        return 1
