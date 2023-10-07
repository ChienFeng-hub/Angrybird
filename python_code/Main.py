from tkinter import *
from Body import *
from BoxVsBox import *
from Arbiter import *
from Parameter import *
from Vect import *
from PIL import Image, ImageTk

def init():

    global canvas, msg, body, en, arbiter, img_l, img_s, img_bird, img_angryBird, img_pig, img_pigg, img_smk0, img_smk1, img_smk2
    canvas = Canvas(root)
    msg = 1
    en = 1
    arbiter = []

    canvas.bind("<ButtonPress-1>", toggle)
    canvas.bind("<B1-Motion>", toggle)
    canvas.bind("<ButtonRelease-1>", release)

    img_landscape = Image.open('landscape.png')
    img_landscape = img_landscape.resize((640, 80))
    img_l = ImageTk.PhotoImage(img_landscape)

    img_slingshot = Image.open('slingshot.jpg')
    img_slingshot = img_slingshot.resize((100, 100))
    img_s = ImageTk.PhotoImage(img_slingshot)

    img_smoke0 = Image.open('smoke.png')
    img_smoke0 = img_smoke0.resize((64, 64))
    img_smk0 = ImageTk.PhotoImage(img_smoke0)

    img_smoke1 = Image.open('smoke1.png')
    img_smoke1 = img_smoke1.resize((64, 64))
    img_smk1 = ImageTk.PhotoImage(img_smoke1)

    img_smoke2 = Image.open('smoke2.png')
    img_smoke2 = img_smoke2.resize((64, 64))
    img_smk2 = ImageTk.PhotoImage(img_smoke2)

    img_angryBird = Image.open('angryBird.png')
    weight, height = img_angryBird.size
    if(weight != 40) & (height != 40):
        img_angryBird = img_angryBird.resize((40, 40))
        img_angryBird.save('angryBird.png')
    img_bird = ImageTk.PhotoImage(img_angryBird)

    img_pigg = Image.open('pig.png')
    weight, height = img_pigg.size
    if(weight != 30) & (height != 30):
        img_pigg = img_pigg.resize((30, 30))
        img_pigg.save('pig.png')
    img_pig = ImageTk.PhotoImage(img_pigg)

    body = []
    body.append(Box(320, 440, 980, 80))
    body.append(Box(380, 400 - h/2, w, h))
    body.append(Box(420, 400 - h/2, w, h))
    body.append(Box(480, 400 - h/2, w, h))
    body.append(Box(450, 400 - h - w/2, h, w))
    # body.append(Box(435, 400 - h - w - h/2, w, h))
    # body.append(Box(465, 400 - h - w - h/2, w, h))
    # body.append(Box(450, 400 - 2*h - w - w/2, h, w))
    body.append(Box(450, 400 - 12.5, 20, 20))
    body.append(Box(org_pos[0], org_pos[1], 32, 32))
    # body.append(Circle(480, 150, 25))
    # body.append(Circle(org_pos[0], org_pos[1], 15)) #must put last one


    body[0].type = "static"
    body[len(body) - 2].type = "pig"
    body[len(body) - 1].name = "bird"
    # body[0].angle = 0.0

    root.bind("<Key>", key)
    root.geometry("640x480")
    canvas.config(bg = "white" ,width = 640 ,height = 480)
    canvas.create_image(0,480,anchor = "sw", image = img_l)
    canvas.create_image(80,400,anchor = "sw", image = img_s)
    # canvas.create_rectangle(recx1,recy1,recx3,recy3)
    canvas.pack()

def toggle(event):
    (x, y) = (event.x, event.y)
    r = 0
    # r = Vector(org_pos[0] - x, org_pos[1] - y).length()
    if r <= sling_len:
        body[len(body) - 1].pos = Vector(x, y)
    else:
        body[len(body) - 1].pos = Vector(sling_len * (x - org_pos[0]) / r + org_pos[0], sling_len * (y - org_pos[1]) / r + org_pos[1])


def release(event):
    global comfirm
    if (recx0 < body[len(body)-1].pos.x) or (recx2 > body[len(body)-1].pos.x) or (recy0 > body[len(body)-1].pos.y) or (recy2 < body[len(body)-1].pos.y):
        comfirm = True
        bird_force(body[len(body) - 1])

comfirm = False

def bird_force(bird):
    dx = bird.pos.x-org_pos[0]
    dy = bird.pos.y-org_pos[1]
    bird.vel.x = coefficient_v * (-dx)
    bird.vel.y = coefficient_v * (-dy)

def key(event):
    global msg, en, body, comfirm
    print(repr(event.char),"is pressed")
    if(str(event.char)=='q'):
        msg = 0
    elif(str(event.char)=='r'):
        msg = 2
        comfirm = False
    elif(str(event.char)=='p'):
        en *= -1
    elif(str(event.char)=='s'):
        body[1].alive = False

def render():
    global body, img_bird, img_pig, img_smk0, img_smk1, img_smk2
    # body[]

    if(not comfirm):
        canvas.delete("cord")
        canvas.create_line(x_a,y_a,body[len(body) - 1].pos.x,body[len(body) - 1].pos.y,x_b,y_b,width = 2, tag = "cord")
    else :
        canvas.delete("cord")

    for i in range(0, len(body)):


        if body[i].shape == "box":
            canvas.delete('box' + str(i))

            if i == len(body) - 1:
                canvas.delete("bird")
                if body[i].alive:
                    img_bird = ImageTk.PhotoImage(img_angryBird.rotate(-body[len(body) - 1].angle * 180/math.pi))
                    canvas.create_image(body[len(body) - 1].pos.x, body[len(body) - 1].pos.y, anchor = 'c', image = img_bird, tag = "bird")
            elif i == len(body) - 2:
                global sm
                canvas.delete("pig")
                canvas.delete("smoke0")
                canvas.delete("smoke1")
                canvas.delete("smoke2")
                if body[i].alive:
                    sm = 0
                    img_bird = ImageTk.PhotoImage(img_pigg.rotate(-body[len(body) - 2].angle * 180/math.pi))
                    canvas.create_image(body[len(body) - 2].pos.x, body[len(body) - 2].pos.y, anchor = 'c', image = img_pig, tag = "pig")
                else:
                    # canvas.delete("smoke")

                    if sm <= 18:
                        if sm <= 6:
                            canvas.create_image(body[len(body) - 2].pos.x, body[len(body) - 2].pos.y, anchor = 'c', image = img_smk0, tag = "smoke0")
                        elif sm <= 12:
                            canvas.create_image(body[len(body) - 2].pos.x, body[len(body) - 2].pos.y, anchor = 'c', image = img_smk1, tag = "smoke1")
                        else:
                            canvas.create_image(body[len(body) - 2].pos.x, body[len(body) - 2].pos.y, anchor = 'c', image = img_smk2, tag = "smoke2")
                        sm += 1

                    else:
                        canvas.delete("smoke0")
                        canvas.delete("smoke1")
                        canvas.delete("smoke2")

            else:
                canvas.delete('box' + str(i))
                if body[i].alive:
                    ver = body[i].getVertices()
                    if i == 0 :
                        canvas.create_polygon(ver[0].x, ver[0].y, ver[1].x, ver[1].y, ver[2].x, ver[2].y, ver[3].x, ver[3].y,
                                      tag = "box" + str(i), fill = '')
                    else:
                        canvas.create_polygon(ver[0].x, ver[0].y, ver[1].x, ver[1].y, ver[2].x, ver[2].y, ver[3].x, ver[3].y,
                                      tag = "box" + str(i), fill = '#825201', outline = 'black')

        # elif body[i].shape == "circle":
        #     canvas.delete('circle' + str(i))
        #
        #     rCir = body[i].radius
        #     CV = [Vector(body[i].pos.x-rCir, body[i].pos.y-rCir), Vector(body[i].pos.x+rCir, body[i].pos.y+rCir)]
        #     canvas.create_oval(CV[0].x, CV[0].y, CV[1].x, CV[1].y, tag = 'circle' + str(i), fill = '')

def BroadPhase():
    global arbiter, body

    for i in range(0, len(body)):

        bi = body[i]

        for j in range(i+1, len(body)):

            bj = body[j]

            if bi.inv_mass() == 0.0 and bj.inv_mass() == 0.0:
                continue
            if not bi.alive or not bj.alive:
                continue

            newArb = Arbiter(bi, bj)

            if newArb.numContacts > 0:
                arbiter.append(newArb)


def Step():
    global body, arbiter
    BroadPhase()

    # print(len(arbiter))
    for i in range(0, len(body)):
        b = body[i]
        if b.inv_mass() == 0.0 or not b.alive:
            continue
        b.vel += (gravity) * dt


    for i in arbiter:
        i.PreStep()

    for i in range(0, itera):
        for j in arbiter:
            j.ApplyImpulse()

    for i in range(0, len(body)):
        b = body[i]

        # print(b.mass, b.inv_mass())

        if b.inv_mass() == 0.0 or not b.alive:
            continue

        b.pos += b.vel * dt
        b.angle += b.angVel * dt

        if b.pos.x > 720 or b.pos.x < -80:
            b.alive = False

    arbiter.clear()

def destroy():
    global body, arbiter
    body.clear()
    arbiter.clear()
    canvas.destroy()


def main():
    global msg, root, en, comfirm

    root = Tk()
    init()

    n = 0
    while msg:
        if en == 1:
            if msg == 2 :
                destroy()
                init()
            if(comfirm):
                Step()
            render()

        root.update()



if __name__ == '__main__':
    main()
