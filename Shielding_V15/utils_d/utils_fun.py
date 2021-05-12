
from gym_env.lunar_lander import (VIEWPORT_H,VIEWPORT_W,SCALE,LEG_DOWN,FPS)

def r3(f):
    return round(f,3)


def display_state(s):
    print("     pos   : ({} | {})".format(r3(s[0]),r3(s[1])))
    print("     speed : ({} | {})".format(r3(s[2]),r3(s[3]))) 
    print("     angle : {}".format(r3(s[4])))
    print("     angular_speed {}".format(r3(s[5])))


# TRANSFORME LA COORDONNEE DE GYM EN BOX 2D
def get_in_Box2D_posx(x):
    # (pos.x - VIEWPORT_W/SCALE/2) / (VIEWPORT_W/SCALE/2),
    cst = VIEWPORT_W/SCALE/2
    return x*cst + cst
def get_in_Box2D_posy(env,y):
    # (pos.y - (self.helipad_y+LEG_DOWN/SCALE)) / (VIEWPORT_H/SCALE/2),
    cst = VIEWPORT_H/SCALE/2
    cst2 = env.helipad_y+LEG_DOWN/SCALE
    return y*cst + cst2

def get_in_Box2D_spx(x):
    # vel.x*(VIEWPORT_W/SCALE/2)/FPS,
    cst = VIEWPORT_W/SCALE/2
    return x*FPS / cst
def get_in_Box2D_spy(y):
    # vel.y*(VIEWPORT_H/SCALE/2)/FPS,
    cst = VIEWPORT_H/SCALE/2
    return y*FPS / cst
def get_in_Box2D_av(av):
    # 20.0*self.lander.angularVelocity/FPS,
    return av*FPS / 20.0