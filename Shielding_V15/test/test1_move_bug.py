import sys
sys.path.append('..')

from gym_env.lunar_lander import LunarLander
import random
from utils_d.discretize import discretize_speed_angle
from Box2D.Box2D import b2Vec2

####################################### BUG #######################################
# env = LunarLander()
# nb_step = 200
# state = env.reset()
# env.lander.position = b2Vec2(10,10)
# for i in range(nb_step):
#     a=0
#     state, r, done, info = env.step(a)
#     env.render()
# env.close()

####################################### FIX #######################################
from utils_d.utils_fun import (get_in_Box2D_posx,get_in_Box2D_posy)
from gym_env.lunar_lander_add_method import reset_to_pos
env = LunarLander()
nb_step = 200
state = env.reset()
posx = get_in_Box2D_posx(0)
posy = get_in_Box2D_posy(env,0.7)
angle = 0
speedx = 0
speedy = 0
angular_vel = 0
reset_to_pos(env, posx, posy, angle, speedx, speedy, angular_vel)
for i in range(nb_step):
    a=0
    state, r, done, info = env.step(a)
    env.render()
env.close()
