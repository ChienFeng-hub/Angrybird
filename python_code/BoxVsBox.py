from Body import *
from Arbiter import *

# Box vertex and edge numbering:
#
#        e0
#   v1 ------ v0
#    |        |
# e1 |        | e3  -- x
#    |        |
#   v2 ------ v3
#        e2
#        |
#        y

def getSupport(dir, body):
    vertex = body.getVertices()
    bestProjection = 0.0
    bestVertex = vertex[0]
    for i in vertex:
        projection = dot(i - body.pos, dir)
        if projection > bestProjection:
            bestProjection = projection
            bestVertex = i

    return bestVertex

def FindLeastPenetration(bodyA, bodyB):
    n = bodyA.getNorms()
    vertexA = bodyA.getVertices()
    bestPenetration = -1000000000
    bestNorm = Vector(0,0)
    bestIndex = 0

    for i in range(0, 4):
        vertexB = getSupport(n[i] * -1, bodyB)
        # print(n[i].x * -1, n[i].y * -1)
        temp = vertexB - vertexA[i]
        distance = dot(temp, n[i])
        # print(distance, vertexB.x, vertexB.y)
        if distance > bestPenetration:
            bestPenetration = distance
            bestNorm = n[i]
            bestIndex = i

    return (bestPenetration, bestNorm, bestIndex)

def ClipSegmentToLine(lineNorm, incidentNorm, contact1, contact2, c1):
    # ax+by+c = 0
    # [ lineNorm.x, lineNorm.y ][ x ] = [ c1 ]
    # [ incident.x, incident.y ][ y ] = [ c2 ]
    # [ x ] = [ incident.y, -lineNorm.y ][ c1 ]
    # [ y ] = [ -incident.x, lineNorm.x ][ c2 ]
    det = lineNorm.x * incidentNorm.y - lineNorm.y * incidentNorm.x
    c2 = dot(incidentNorm, contact1.pos)
    newPos = Vector(0,0)
    newPos.x =  (incidentNorm.y*c1 - lineNorm.y*c2)/det
    newPos.y =  (lineNorm.x*c2 - incidentNorm.x*c1)/det

    lineNormIn = lineNorm * -1
    newPosToCon1 = contact1.pos - newPos
    newPosToCon2 = contact2.pos - newPos
    product1 = dot(newPosToCon1, lineNormIn)
    product2 = dot(newPosToCon2, lineNormIn)


    if product1 < 0:
        contact1.pos = newPos
        contact1.normal = contact2.normal
    elif product2 < 0:
        contact2.pos = newPos
        contact2.normal = contact1.normal
    else:
        return


def ComputeIncidentEdge(body, referenceNorm):
    n = body.getNorms()
    smallestDotProduct = 0
    incidentIndex = 0
    for i in range(0, 4):
        temp = dot(n[i], referenceNorm)
        if temp < smallestDotProduct:
            smallestDotProduct = temp
            incidentIndex = i

    return incidentIndex, n[incidentIndex]

def Collide(Contacts, bodyA, bodyB):
    queryA = FindLeastPenetration(bodyA, bodyB)
    if queryA[0] > 0.0:
        return 0

    queryB = FindLeastPenetration(bodyB, bodyA)
    if queryB[0] > 0.0:
        return 0


    penetration_A = queryA[0]
    penetration_B = queryB[0]

    flip = False
    contact1 = Contact()
    contact2 = Contact()

    if penetration_A >= penetration_B:
        vertice = bodyA.getVertices()
        referenceNorm = queryA[1]
        referenceIndex = queryA[2]
        incidentIndex, incidentNorm = ComputeIncidentEdge(bodyB, referenceNorm)
        side1Norm = bodyA.getNorms()[(referenceIndex-1)%4]
        side2Norm = bodyA.getNorms()[(referenceIndex+1)%4]
    else :
        vertice = bodyB.getVertices()
        referenceNorm = queryB[1]
        referenceIndex = queryB[2]
        incidentIndex, incidentNorm = ComputeIncidentEdge(bodyA, referenceNorm)
        side1Norm = bodyB.getNorms()[(referenceIndex-1)%4]
        side2Norm = bodyB.getNorms()[(referenceIndex+1)%4]
        flip = True

    if flip:
        v = bodyA.getVertices()
        contact1.pos = v[incidentIndex]
        contact1.normal = referenceNorm * -1

        contact2.pos = v[(incidentIndex+1)%4]
        contact2.normal = referenceNorm * -1

    else:
        v = bodyB.getVertices()
        contact1.pos = v[incidentIndex]
        contact1.normal = referenceNorm

        contact2.pos = v[(incidentIndex+1)%4]
        contact2.normal = referenceNorm

    c1 = dot(side1Norm, vertice[(referenceIndex-1)%4])
    ClipSegmentToLine(side1Norm, incidentNorm, contact1, contact2, c1)
    c1 = dot(side2Norm, vertice[(referenceIndex+1)%4])
    ClipSegmentToLine(side2Norm, incidentNorm, contact1, contact2, c1)


    #reference clip
    referVertex = vertice[referenceIndex%4]
    product1 = dot(contact1.pos - referVertex, referenceNorm * -1)
    product2 = dot(contact2.pos - referVertex, referenceNorm * -1)

    if product1 >= 0 and product2 >= 0:
        numContacts = 2
        contact1.separation = -product1
        contact2.separation = -product2
        Contacts.append(contact1)
        Contacts.append(contact2)
        # numContacts = 1
        # if product1 >= product2:
        #     contact1.separation = -product1
        #     Contacts.append(contact1)
        # else:
        #     contact2.separation = -product2
        #     Contacts.append(contact2)

    elif product1 >= 0 and product2 < 0:
        numContacts = 1
        contact1.separation = -product1
        Contacts.append(contact1)
    elif product1 < 0 and product2 >= 0:
        numContacts = 1
        contact2.separation = -product2
        Contacts.append(contact2)
    else:
        return 0

    return numContacts



