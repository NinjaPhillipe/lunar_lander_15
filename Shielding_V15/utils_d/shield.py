import pickle 

from utils.discretize import GLOBAL_STATE

PATH = "output/"

PATH_SHIELD = PATH+"shield.dump"

PATH_TRANSITION = PATH+"transition_file"
PATH_LABEL = PATH+"labeling_file"

def load_shield():
    with open( PATH_SHIELD,'rb') as f:
        return pickle.load(f)

"""
dic {
                2  4
    s1 = [dic{s1',s2'},dic{s1',s2'},dic{s1',s2'},dic{s1',s2'}] 
    s1 = {}

}
"""
class Shield():

    def __init__(self):
        self.states = {}
        self.deadStates = {}
        self.globalS = 0

    def add_dead_state(self,n_s):
        if n_s is not None:
            self.deadStates[n_s] = True

    def add_transition(self,n_s1,n_s2,a):
        # Si n'est pas dans le shield
        if n_s1 == None or n_s2 == None:
            return False
        
        # Pas de transition depuis l'etat global
        if n_s1 == GLOBAL_STATE:
            self.globalS +=1
            return False

        # Si pas deja dans les etats explo
        if n_s1 not in self.states.keys():
            self.states[n_s1] = [{} for i in range(4)]

        # Si pas encore de transition vers cet etat
        if n_s2 not in self.states[n_s1][a].keys():
            self.states[n_s1][a][n_s2] = 0

        self.states[n_s1][a][n_s2] += 1

        return True
    def dump(self,name):
        with open(PATH+name+".dump", 'wb') as f:
            pickle.dump(self, f)

    def to_explicit_file(self):
        with open(PATH_TRANSITION, "w") as f:
            f.write("mdp \n \n")
            for key in sorted(self.states): # pour chaque états
                for i in range (4): # pour chaque dir de l'états
                    res = 0
                    dic = self.states[key][i]
                    for next_state in dic:
                        res += dic[next_state]
                    proba = 0
                    for next_state in dic:
                        proba = dic[next_state] / res

                        line  = str(key)+" "+str(i)+" "+str(next_state)+" "+str(proba)
                        f.write("" + line + "\n")
        #Fichier des labels
        with open(PATH_LABEL, "w") as f:
            f.write("#DECLARATION \n")
            f.write("deadlock \n")
            f.write("#END \n")
            for key in sorted(self.deadStates):
                f.write("{} deadlock \n".format(key))