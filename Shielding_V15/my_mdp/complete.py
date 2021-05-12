import stormpy 
import numpy as np

from gym_env.lunar_lander import (LunarLander,VIEWPORT_H,VIEWPORT_W,SCALE,LEG_DOWN,FPS)
from gym_env.lunar_lander_add_method import reset_to_pos_saucer
from gym_env.saucer import LanderObstacle
from utils_d.mdp import Mdp
from utils_d.utils_fun import *
from utils_d.discretize import (Bounds,discretize_complete,get_bounds_array,LABEL_DEADLOCK,LABEL_HITSAUCER,LABEL_GOAL)
from my_mdp.mdp_bounds import complete_bounds

COMPLETE_NAME = "completemdp"


def isgoal(sp):
    # position
    if(sp[0]<-0.1 or 0.1 < sp[0]):
        return False
    if(sp[1]<-0.05 or 0.1 < sp[1]):
        return False
    
    # vistesse
    if(sp[2]<-0.1 or 0.1 < sp[2]):
        return False

    if(sp[3]<-0.01 or 0.01 < sp[3]):
        return False

    # angle 
    if(sp[4]<-0.1 or 0.1 < sp[4]):
        return False
    if(sp[5]<-0.05 or 0.05 < sp[5]):
        return False
    
    return True

def train_complete_mdp(bounds,load=False,nb_data=10,render=False):
    """
        nb_data : # nombre d´echantillon
    """
    env = LanderObstacle()

    mdp = None 
    try:
        mdp = Mdp(COMPLETE_NAME,load)
    except:
        print("NO MDP FILE FOUND")
        mdp = Mdp(COMPLETE_NAME,False)

    bpx = bounds[0]
    bpy = bounds[1]
    bsx = bounds[2]
    bsy = bounds[3]
    ba  = bounds[4]

    ech = 5
    max_rep = 5

    count = 0

    nb_data_angle = int(nb_data/2)

    # add big danger left training 
    tot_ech = nb_data**4 * nb_data_angle

    # pos
    px_l = np.random.uniform(low=bpx.big_danger_l, high =bpx.big_danger_r, size=nb_data)
    py_l = np.random.uniform(low=bpy.big_danger_l, high =bpy.big_danger_r, size=nb_data)

    # Sampling de valeur par rapport a la range du shield
    sx_l = np.random.uniform(low=bsx.big_danger_l, high =bsx.big_danger_r, size=nb_data)
    sy_l = np.random.uniform(low=bsy.big_danger_l, high =bsy.big_danger_r, size=nb_data)
    an_l = np.random.uniform(low=ba.big_danger_l,  high =ba.big_danger_r,  size=nb_data_angle)
    av_l = np.random.uniform(low=-0.1,  high =0.1,  size=nb_data_angle) # TODO 0.1 a la place de 0.2

    # pour chaque position
    for px in px_l:
        for py in py_l:
            # pour chaque stabilité
            for sx in sx_l:
                for sy in sy_l:
                    for angle in range(nb_data_angle):
                        av = av_l[angle]
                        an = an_l[angle]

                        count+=1

                        if count %100 ==0:
                            print(" {}%".format(round(count/tot_ech*100,3) ))

                        # Effectuer ech fois pour chaque action
                        for i in range(ech):
                            for a in [0,1,2,3]:

                                # PLACE LE LUNAR LANDER DANS L'ETAT SOUHAITER
                                posx = get_in_Box2D_posx(px)
                                posy = get_in_Box2D_posy(env,py)
                                angle = an
                                speedx = get_in_Box2D_spx(sx)
                                speedy = get_in_Box2D_spy(sy)
                                angular_vel = get_in_Box2D_av(av)
                                s = reset_to_pos_saucer(env, posx, posy, angle, speedx, speedy,angular_vel)

                                # fait m etape pour ne pas avoir de self loop
                                sp, r, done, info = env.step(a)
                                (n_s,label_s)  = discretize_complete(s,bounds)
                                (n_sp,label_sp) = discretize_complete(sp,bounds)

                                if render:
                                    env.render()
                                # effecture les action jusqu'a atteindre un etat different ou le max d'action
                                for _ in range(max_rep):
                                    if(env.hitSaucer ):
                                        mdp.add_label(n_sp, LABEL_HITSAUCER)
                                        break
                                    if n_s != n_sp:
                                        break
                                    sp, r, done, info = env.step(a)
                                    (n_sp,label_sp) = discretize_complete(sp,bounds)
                                    # env.render()

                                mdp.add_transition(n_s, n_sp, a)
                                mdp.add_label(n_sp, label_sp)

                                # verifie si c'est un etat goal 
                                if isgoal(sp):
                                    mdp.add_label(n_sp, LABEL_GOAL)
    env.close()
    return mdp


def train_model_complete(render = False,nb_data=10):

    # prends les bornes
    bounds = complete_bounds() 

    # fait une etape d'entrainement
    complete = train_complete_mdp(bounds,load=True,nb_data=nb_data,render=render)

    # sauvegarde
    complete.dump()
    complete.to_explicit_file()
    complete.to_prism_file()


def load_scheduler_mdp_complete(prop):
    complete = Mdp(COMPLETE_NAME)

    model = stormpy.build_sparse_model_from_explicit(transition_file=complete.PATH_TRANSITION, labeling_file=complete.PATH_LABEL)
    print(model)

    properties = stormpy.parse_properties_without_context(prop)

    result = stormpy.model_checking(model, properties[0],only_initial_states=False, extract_scheduler=True)

    assert result.has_scheduler
    scheduler = result.scheduler
    assert scheduler.memoryless
    assert scheduler.deterministic
    print(scheduler)
    return (result,scheduler,complete_bounds())
