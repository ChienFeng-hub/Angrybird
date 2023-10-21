import Vect
import math
fps = 10
dt = 1/fps
inv_dt = fps
Vel = 12000
gravity = Vect.Vector(0, 12)
elastic = 0
friction = 0.3
itera = 30
coefficient_v = 1.5
sling_len = 64
org_pos = (130, 310)
recx0,recx2 = 150,110
recy0,recy2 = 290,330
x_a, y_a, x_b, y_b = 111, 310, 144, 310

sm = 0

w = 8
h = 64
