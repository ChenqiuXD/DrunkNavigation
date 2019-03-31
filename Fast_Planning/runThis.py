from Classes import *

if __name__ == "__main__":
    a = Point(0, 0)
    b = Point(200, 200)
    c = Point(400, 400)

    pqx = c.x - a.x
    pqy = c.y - a.y
    dx = b.x - a.x
    dy = b.y - a.y
    d = pqx*pqx+pqy*pqy
    t = pqx*dx+pqy*dy
    if t<0:
        t = 0
    elif t>1:
        t = 1

    dx = a.x + t*pqx - b.x
    dy = a.y + t*pqy - b.y

    print(dx*dx+dy*dy)


