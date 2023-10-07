from Vect import *
from Body import *
from Parameter import *
import BoxVsBox
import CircleVsBox
import CircleVsCircle
import math

class Contact:
    def __init__(self):
        self.pos = Vector(0,0)
        self.normal = Vector(0,0)
        self.separation = 0.0
        self.r1, self.r2 = Vector(0,0), Vector(0,0)
        self.Pn = 0.0
        self.Pt = 0.0
        self.massNormal, self.massTangent = 0.0, 0.0
        self.bias = 0.0


class Arbiter():
    def __init__(self, body1, body2):

        self.contacts = []
        if body1.shape == "circle" and body2.shape == "circle":
            if body1.pos.y > body2.pos.y:
                self.body1 = body1
                self.body2 = body2
            else:
                self.body1 = body2
                self.body2 = body1

            self.numContacts = CircleVsCircle.Collide(self.contacts, self.body1, self.body2)

        elif body1.shape == "circle":
            self.body1 = body1
            self.body2 = body2
            self.numContacts = CircleVsBox.Collide(self.contacts, self.body1, self.body2)

        elif body2.shape == "circle":
            self.body1 = body2
            self.body2 = body1
            self.numContacts = CircleVsBox.Collide(self.contacts, self.body1, self.body2)

        else:
            # if body1.pos.y > body2.pos.y:
            #     self.body1 = body1
            #     self.body2 = body2
            # else:
            #     self.body1 = body2
            #     self.body2 = body1
            self.body1 = body2
            self.body2 = body1
            self.numContacts = BoxVsBox.Collide(self.contacts, self.body1, self.body2)



    def PreStep(self):
        k_allowedPenetration = 0.125
        k_biasFactor = 0.25
        for i in range(0, self.numContacts) :
            c = self.contacts[i]
            r1 = c.pos - self.body1.pos
            r2 = c.pos - self.body2.pos

            #Precompute normal mass, tangent mass, and bias.
            rn1 = dot(r1, c.normal)
            rn2 = dot(r2, c.normal)
            kNormal = self.body1.inv_mass() + self.body2.inv_mass()
            kNormal += self.body1.inv_I() * (dot(r1, r1) - rn1 * rn1) \
                       + self.body2.inv_I() * (dot(r2, r2) - rn2 * rn2)
            c.massNormal = 1.0 / kNormal

            tangent = vectorXscalar(c.normal, 1.0)
            rt1 = dot(r1, tangent)
            rt2 = dot(r2, tangent)
            kTangent = self.body1.inv_mass() + self.body2.inv_mass()
            kTangent += self.body1.inv_I() * (dot(r1, r1) - rt1 * rt1) \
                        + self.body2.inv_I() * (dot(r2, r2) - rt2 * rt2)
            c.massTangent = 1.0/kTangent

            c.bias = -k_biasFactor * inv_dt * min(0.0, c.separation + k_allowedPenetration)

            # print(c.separation, c.pos.x, c.pos.y, c.normal.x, c.normal.y, self.body1.pos.x, self.body1.pos.y
            #       , self.body2.pos.x, self.body2.pos.y, self.body1.inv_mass(), self.body2.inv_mass(), self.body1.inv_I(), self.body2.inv_I())
            # print("----")
            # print(c.massTangent, c.massNormal, kNormal, kTangent, c.bias)
            # print("---------")

            # P = c.normal * c.Pn + tangent * c.Pt
            # self.body1.vel -= P * self.body1.inv_mass()
            # self.body1.angVel -= self.body1.inv_I() * cross(r1, P)
            #
            # self.body2.vel += P * self.body2.inv_mass()
            # self.body2.angVel -= self.body2.inv_I() * cross(r2, P)


    def ApplyImpulse(self):
        b1 = self.body1
        b2 = self.body2

        for i in range(0, self.numContacts):
            c = self.contacts[i]
            c.r1 = c.pos - b1.pos
            c.r2 = c.pos - b2.pos

            # Relative.vel
            dv = b2.vel + scalarXvector(b2.angVel, c.r2) \
                 - b1.vel - scalarXvector(b1.angVel, c.r1)

            # Compute Normal impulse
            vn = dot(dv, c.normal)
            dPn = c.massNormal * (-vn + c.bias)

            # print(dPn)
            if dPn >= 96 or dPn <= -96:
                if b1.type == "pig":
                    b1.alive = False
                elif b2.type == "pig":
                    b2.alive = False

            # Clamp the accumulated impulse
            # Pn0 = c.Pn
            # c.Pn = max(Pn0 + dPn, 0.0)
            # dPn = max(c.Pn - Pn0, 0.0)
            dPn = max(dPn, 0.0)

            Pn = c.normal * dPn

            b1.vel -= Pn * self.body1.inv_mass()
            b1.angVel -= self.body1.inv_I() * cross(c.r1, Pn)
            #
            b2.vel += Pn * self.body2.inv_mass()
            b2.angVel += self.body2.inv_I() * cross(c.r2, Pn)

            dv = b2.vel + scalarXvector(b2.angVel, c.r2) \
                 - b1.vel - scalarXvector(b1.angVel, c.r1)

            tangent = vectorXscalar(c.normal, 1.0)
            vt = dot(dv, tangent)
            dPt = c.massTangent * (-vt)

            maxPt = friction * c.Pn
            # oldTangentImpulse = c.Pt
            # c.Pt = clamp(oldTangentImpulse + dPt, -maxPt, maxPt)
            # dPt = c.Pt - oldTangentImpulse
            dPt = clamp(dPt, -maxPt, maxPt)

            Pt = tangent * dPt

            b1.vel -= Pt * self.body1.inv_mass()
            b1.angVel -= self.body1.inv_I() * cross(c.r1, Pt)
            #
            b2.vel += Pt * self.body2.inv_mass()
            b2.angVel += self.body2.inv_I() * cross(c.r2, Pt)

            # print(Pn.x, Pn.y, cross(c.r1, Pn))
            # print(dPn, dPt, c.Pn, c.Pt)
            # print(b1.vel.x, b1.vel.y, b1.angVel, b1.name)





    # def PositionalCorrection(self):
    #     k_slop = 0.01
    #     percent = 0.2
    #     for i in range(0, self.numContacts):
    #         c = self.contacts[i]
    #         correction = -min(0.0, c.separation + k_slop) \
    #                      / (self.body1.inv_mass() + self.body2.inv_mass()) * percent
    #         correction = c.normal * correction
    #         self.body1.pos -= correction * self.body1.inv_mass()
    #         self.body2.pos += correction * self.body2.inv_mass()




