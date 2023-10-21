from Body import *
import Arbiter
import math

def Collide(Contacts, cir, box):
    radiiToVertex = 100000000
    bestIndex = 0
    radius = cir.radius
    V = box.getVertices()
    norm = box.getNorms()

    for i in range(0, 4):
        temp = dot((cir.pos - V[i]), (cir.pos - V[i]))
        if temp < radiiToVertex:
            radiiToVertex = temp
            bestIndex = i

    sideNorm1 = norm[(bestIndex-1)%4]
    sideNorm2 = norm[bestIndex]

    sideProj1 = dot(sideNorm1, (cir.pos - V[bestIndex]))
    sideProj2 = dot(sideNorm2, (cir.pos - V[bestIndex]))

    if sideProj1 > radius or sideProj2 > radius:
        return 0

    if sideProj1 >= sideProj2:
        if sideProj2 <= 0:
            bestIndex = (bestIndex-1)%4
            Norm = norm[bestIndex]
            contactPoint = cir.pos + (Norm * -1) * radius
            penetration = radius - sideProj1
        else:
            distance = math.sqrt(radiiToVertex)
            if distance > radius:
                return 0
            Norm = (cir.pos - V[bestIndex]).normalize()
            contactPoint = cir.pos + (Norm * -1) * radius
            penetration = radius - distance
    else:
        if sideProj1 <= 0:
            bestIndex = (bestIndex)%4
            Norm = norm[bestIndex]
            contactPoint = cir.pos + (Norm * -1) * radius
            penetration = radius - sideProj2
        else:
            distance = math.sqrt(radiiToVertex)
            if distance > radius:
                return 0
            Norm = (cir.pos - V[bestIndex]).normalize()
            contactPoint = cir.pos + (Norm * -1) * radius
            penetration = radius - distance

    contact1 = Arbiter.Contact()
    contact1.normal = Norm * -1
    contact1.separation = -penetration
    contact1.pos = contactPoint
    Contacts.append(contact1)

    return 1
