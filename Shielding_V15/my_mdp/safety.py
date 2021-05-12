import sys
sys.path.append('..')

import stormpy 
import numpy as np

from gym_env.lunar_lander import (LunarLander,VIEWPORT_H,VIEWPORT_W,SCALE,LEG_DOWN,FPS)
from gym_env.lunar_lander_add_method import reset_to_pos
from utils_d.mdp import Mdp
from utils_d.utils_fun import *
from utils_d.discretize import (Bounds,discretize_speed_angle,get_bounds_array,LABEL_DEADLOCK)
from my_mdp.mdp_bounds import get_stab_bounds



SAFETY_NAME ="safetyasxy"

def train_shield_safety(stab,load=False,nb_data=10,render=False):
    """
        nb_data : # nombre d´echantillon
    """

    bsx = stab[0]
    bsy = stab[1]
    ba  = stab[2]

    env = LunarLander()

    mdp = None 
    try:
        mdp = Mdp(SAFETY_NAME,load)
    except:
        print("NO MDP FILE FOUND")
        mdp = Mdp(SAFETY_NAME,False)

    ech = 5
    max_rep = 5

    count = 0

    # add big danger left training 
    tot_ech = nb_data**3

    # Sampling de valeur par rapport a la range du shield
    sx_l = np.random.uniform(low=bsx.big_danger_l, high =bsx.big_danger_r, size=nb_data)
    sy_l = np.random.uniform(low=bsy.big_danger_l, high =bsy.big_danger_r, size=nb_data)
    an_l = np.random.uniform(low=ba.big_danger_l,  high =ba.big_danger_r,  size=nb_data)
    av_l = np.random.uniform(low=-0.2,  high =0.2,  size = nb_data)

    for sx in sx_l:
        for sy in sy_l:
            for angle in range(nb_data):
                av = av_l[angle]
                an = an_l[angle]

                count+=1

                if count %100 ==0:
                    print(" {}%".format(round(count/tot_ech*100,3) ))

                # Effectuer ech fois pour chaque action
                for i in range(ech):
                    for a in [0,1,2,3]:

                        # PLACE LE LUNAR LANDER DANS L'ETAT SOUHAITER
                        posx = get_in_Box2D_posx(0)
                        posy = get_in_Box2D_posy(env,0.7)
                        angle = an
                        speedx = get_in_Box2D_spx(sx)
                        speedy = get_in_Box2D_spy(sy)
                        angular_vel = get_in_Box2D_av(av)
                        s = reset_to_pos(env, posx, posy, angle, speedx, speedy,angular_vel)

                        # fait m etape pour ne pas avoir de self loop
                        sp, r, done, info = env.step(a)
                        (n_s,label_s)  = discretize_speed_angle(s,stab)
                        (n_sp,label_sp) = discretize_speed_angle(sp,stab)

                        if render:
                            env.render()
                        # effecture les action jusqu'a atteindre un etat different ou le max d'action
                        for _ in range(max_rep):
                            if n_s != n_sp:
                                break
                            sp, r, done, info = env.step(a)
                            (n_sp,label_sp) = discretize_speed_angle(sp,stab)
                            # env.render()

                        mdp.add_transition(n_s, n_sp, a)
                        mdp.add_label(n_sp, label_sp)
    env.close()
    return mdp

def train_model_safety(render = False,nb_data=10):

    stab = get_stab_bounds()

    safety = train_shield_safety(stab,load=True,nb_data=nb_data,render=render)

    safety.dump()
    safety.to_explicit_file()

def load_scheduler_mdp_safety():

    safety = Mdp(SAFETY_NAME)

    model = stormpy.build_sparse_model_from_explicit(transition_file=safety.PATH_TRANSITION, labeling_file=safety.PATH_LABEL)
    print(model)

    prop = "Tmin=? [ F \"safe\" ]"
    properties = stormpy.parse_properties_without_context(prop)

    result = stormpy.model_checking(model, properties[0],only_initial_states=False, extract_scheduler=True)

    assert result.has_scheduler
    scheduler = result.scheduler
    assert scheduler.memoryless
    assert scheduler.deterministic
    print(scheduler)

    return (result,scheduler,get_stab_bounds())

