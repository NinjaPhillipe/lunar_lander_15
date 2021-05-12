import pickle 

from utils_d.discretize import GLOBAL_STATE,LABEL_GOAL,LABEL_HITSAUCER,LABEL_SAFE,LABEL_DANGER,LABEL_BIG_DANGER

PATH = "output/"



def load_mdp():
    with open( PATH_SHIELD,'rb') as f:
        return pickle.load(f)

"""
dic {
                2  4
    s1 = [dic{s1',s2'},dic{s1',s2'},dic{s1',s2'},dic{s1',s2'}] 
    s1 = {}

}
"""
class Mdp():

    def __init__(self,name,load=False):
        self.name = name
        self.PATH_SHIELD = PATH+name+"mdp.dump"

        self.PATH_TRANSITION = PATH+name+"mdp_transition_file"
        self.PATH_LABEL = PATH+name+"mdp_labeling_file"
        self.PATH_PRISM = PATH+name+"prism.nm"

        self.maxID = 0
        
        if load:
            with open( self.PATH_SHIELD,'rb') as f:
                print("LOAD_SHIELD")
                tmp = pickle.load(f)
                self.states = tmp.states
                self.labels = tmp.labels
        else:
            print("NEW_SHIELD")
            self.states = {}
            self.labels = {}

    def add_label(self,n_s,label):
        if n_s is not None and label is not None:
            # if not n_s in self.labels.keys(): # si pas deja assigner
            # self.labels[n_s] = label
        # TODO verifier si liste necessaire
            if n_s in self.labels.keys():
                if not label in self.labels[n_s]:
                    self.labels[n_s].append(label)
            else:
                self.labels[n_s] = [label]

    def add_transition(self,n_s1,n_s2,a):
        # Si n'est pas dans le shield
        if n_s1 == None or n_s2 == None:
            return False
        
        # Pas de transition depuis l'etat global
        if n_s1 == GLOBAL_STATE:
            return False

        # Si pas deja dans les etats explo
        if n_s1 not in self.states.keys():
            self.states[n_s1] = [{} for i in range(4)]

        # Si pas encore de transition vers cet etat
        if n_s2 not in self.states[n_s1][a].keys():
            self.states[n_s1][a][n_s2] = 0

        self.states[n_s1][a][n_s2] += 1

        # garde en memoire la taille max
        if n_s1 > self.maxID:
            maxID = n_s1
        if n_s2 > self.maxID:
            maxID = n_s2


        return True
    def dump(self):
        with open(self.PATH_SHIELD, 'wb') as f:
            pickle.dump(self, f)

    def to_explicit_file(self):
        with open(self.PATH_TRANSITION, "w") as f:
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
        with open(self.PATH_LABEL, "w") as f:
            f.write("#DECLARATION \n")
            f.write("goal danger bigdanger safe deadlock hit \n")
            f.write("#END \n")
            for key in sorted(self.labels):
                

                # f.write("{} {} \n".format(key,self.labels[key] ))
                # VERSION LISTE 
                f.write("{} {} \n".format(key," ".join(self.labels[key]) ))

    
    def get_states_nb(self):
        tmp = None
        for i in self.states.keys():
            if tmp is None or i > tmp:
                tmp = i
        return tmp
    
    def _label(self,lab):
        line = "label \"{}\" = ".format(lab)
        sep = None
        count = 0
        for i,k in enumerate(self.labels):
            if lab in self.labels[k]:
                # AJOUT SEPARATEUR
                if sep is None: 
                    sep = "|"
                else:
                    line += sep 

                line += "(s = {})".format(k)
                count += 1
                if count > 20:
                    line += "\n"
                    count = 0

        if sep is None: # si aucun label n'a été trouver
            return ""
        return line+";\n"

    def to_prism_file(self):
        to_txt = { 0:"one", 1:"two",2:"three",3:"four"}
        with open(self.PATH_PRISM,"w") as f:
            f.write("mdp \n")
            
            

            f.write("module m1 \n")
            f.write("    s : [{}..{}]; \n".format(0,self.get_states_nb()))

            for key in sorted(self.states): # pour chaque états
                for i in range (4): # pour chaque dir de l'états

                    res = 0
                    dic = self.states[key][i]
                    for next_state in dic:
                        res += dic[next_state]
                    # proba = 0

                    count = 0
                    if len(dic.keys()) > 0: # Si il esxiste au moins un etat vers lequel effectuer une transition
                        line = "    [{}] s={} -> ".format(to_txt[i],key)
                        for i,next_state in enumerate(dic):
                            # proba = dic[next_state] / res
                            line += "{}/{}:(s' = {})".format(dic[next_state],res,next_state)
                            if(i < len(dic)-1):
                                line+=" + "
                            count +=1 
                            if count > 20:
                                line += "\n"
                                count = 0
                        f.write(line+"; \n")

            f.write("endmodule \n\n")

            # line = "init "
            # sep = None
            # for k in self.states.keys():
            #     # AJOUT SEPARATEUR
            #     if sep is None: 
            #         sep = "|"
            #     else:
            #         line += sep 
            #     line += " s = {} ".format(k)
            # f.write(line+" endinit\n")

            f.write(self._label(LABEL_GOAL))
            f.write(self._label(LABEL_HITSAUCER))

            f.write(self._label(LABEL_SAFE))
            f.write(self._label(LABEL_DANGER))
            f.write(self._label(LABEL_BIG_DANGER))
            
            
            f.write("\n")    