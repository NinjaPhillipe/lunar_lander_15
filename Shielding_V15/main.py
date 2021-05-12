import stormpy 
import random
import json



from time import sleep
from Box2D.Box2D import b2Vec2

from my_mdp.safety import (train_model_safety,load_scheduler_mdp_safety,SAFETY_NAME)
from my_mdp.complete import (train_model_complete,load_scheduler_mdp_complete,COMPLETE_NAME)
from my_mdp.hit_mdp import (train_model_hit,load_scheduler_mdp_hit,HIT_NAME)

from utils_d.mdp import (Mdp)
from utils_d.utils_fun import *
from utils_d.discretize import Bounds,size_bounds,inbounds
from utils_d.discretize import (LABEL_DEADLOCK,LABEL_SAFE,LABEL_DANGER,LABEL_BIG_DANGER)
from utils_d.discretize import (discretize_speed_angle,discretize_complete)

from gym_env.lunar_lander import LunarLander
from gym_env.saucer import LanderObstacle
from gym_env.lunar_lander_add_method import reset_to_pos,reset_to_pos_saucer

####################################################################################################################################
######################################################## OTHER #####################################################################
####################################################################################################################################

def exec_com(env,res_sta,res_hit,res_goa,render=False,log=False):
    
    nb_step = 800
    
    state = env.reset()


    #        ACTIONS
    #
    #         3   1
    #           2    


    total_reward = 0

    for i in range(nb_step):
        # sleep(0.05)
        (s_ns,s_label) = discretize_speed_angle(state,res_sta[2])
        (h_ns,h_label) = discretize_complete(state,res_hit[2])
        (g_ns,g_label) = discretize_complete(state,res_goa[2])


        ###########################################
        #### strategie en dehors des trois mdp ####
        ###########################################
        a = None 
    
        if state[2] < res_sta[2][0].big_danger_l:
            a = 3
        elif res_sta[2][0].big_danger_r < state[2]:
            a = 1

        if a == None:
            if state[3] < res_sta[2][1].big_danger_l:
                a = 2
            elif res_sta[2][1].big_danger_r < state[3]:
                a = 0
        if a == None:
            if state[4] < res_sta[2][2].big_danger_l:
                a = 1
            elif res_sta[2][2].big_danger_r < state[4]:
                a = 3
        ###########################################
        # if log:
        #     print(state)
        if a is None:
            if inbounds(state,res_hit[2]) and res_hit[0].get_values()[h_ns] < 1.0:
                a = res_hit[1].get_choice(h_ns).get_deterministic_choice()
                if log:
                    print("HIT MDP ")

            else:
                if inbounds(state,res_goa[2]):
                    a = res_goa[1].get_choice(g_ns).get_deterministic_choice()
                    if log:
                        print("GOAL MDP ")
                else:
                    if log:
                        print("TODO default strat")

                    if s_label == LABEL_SAFE or (s_label == LABEL_DANGER and random.random() > 0.8 ):
                        if state[0] < -0.1:
                            a = 3
                        elif state[0] > 0.1:
                            a = 1
                        else:
                            a = 0
                    else:
                        a = res_sta[1].get_choice(s_ns).get_deterministic_choice()


        state, r, done, info = env.step(a)
        total_reward += r

        if render:
            env.render()

        if done:
            break

    return total_reward

import matplotlib.pyplot as plt

from my_mdp.mdp_bounds import printsize
if __name__ == "__main__":
    # printsize()
    #############################################################################################################################
    ########################################################### TRAIN ###########################################################
    #############################################################################################################################
    train = False
    if (train):
        nb_cycle = 7
        for i in range(nb_cycle):
            train_model_safety(render=False,nb_data=20)



    #TODO essayer de transformer hit en deadlock  

    train_complete = False
    if(train_complete):
        nb_cycle = 45
        for i in range(nb_cycle):
            train_model_complete(render=False,nb_data=6)

    train_hit = False
    if (train_hit):
        nb_cycle = 45
        for i in range(nb_cycle):
            train_model_hit(render=False,nb_data=6)


    #############################################################################################################################
    ###########################################################  TEST  ##########################################################
    #############################################################################################################################
    # sc_pos = load_scheduler_mdp_position("Pmax=? [  F \"goal\" ]")

    # (res_com,sc_com) = load_scheduler_mdp_complete("Pmax=? [ G !\"hit\" ]")


    test = True
    if(test):

        res_sta = load_scheduler_mdp_safety()
        res_hit = load_scheduler_mdp_hit("Pmax=? [ G !\"hit\" ]")
        res_goa = load_scheduler_mdp_complete("Tmin=? [ F \"goal\" ]")

        # print(res_hit[0].get_values())
        # exit()
        rewards = []
        
        env = LanderObstacle()
        
        for i in range (100):
            print(i)
            # r = step(env,sc_saf=sc_saf,sc_pos=None,sc_com=sc_com,test=False,render=True,log=False)
            r = exec_com(env,res_sta,res_hit,res_goa,render=False,log=False)
            print(r)
            rewards.append(r)

        env.close()

        plt.plot(rewards)
        plt.show()

    # storm --prism positionxyprism.nm --prop "Pmax=? [ (! \"hit\")  U \"goal\" ]" --exportscheduler "sc.json" --debug
