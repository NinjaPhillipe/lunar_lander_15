
# Borne shield
x  = -0.3
xp =  0.3

ypp = 0.4

# haut shield
y = 1.0

x_step = 0.05
x_range = round((xp*2)/x_step)

x_inf = round(x /x_step)
x_sup = round(xp/x_step)

y_step = 0.05
y_range = round((y-ypp)/y_step)

y_inf = round(ypp/y_step)
y_sup = round(y/ y_step)


GLOBAL_STATE = x_range + (y_range*(x_range+1)) +1



from math import ceil

def discretize(s):
    x = round((s[0]) / x_step)
    y = round((s[1]) / y_step)

    if ( x_inf <= x and x <= x_sup and y_inf <= y and y <= y_sup ):
        x-=x_inf
        y-=y_inf
        return x + y*(x_range+1)
    return GLOBAL_STATE

# def discr_before(b1,b2,i,bounds):
#     return round( abs(b1-b2) / bounds.disc[i] )




############################################################################################
######################################## LABELS ############################################
############################################################################################

LABEL_SAFE = "safe"
LABEL_DANGER = "danger"
LABEL_BIG_DANGER = "bigdanger"
LABEL_DEADLOCK = "deadlock"
LABEL_GOAL = "goal"
LABEL_HITSAUCER =  "hit"


############################################################################################
#################################### Struct Bounds #########################################
############################################################################################

class Bounds:
    """
        Struct qui représente les différentes zone d'une discretisation
        3 ZONE 
           SAFE
           DANGER
           BIG DANGER
    """
    def __init__(self,bdl,dl,sl,sr,dr,bdr,disc):
        self.big_danger_l = bdl
        # BIG DANGER
        self.danger_l = dl
        # DANGER
        self.safe_l = sl
        #SAFE
        self.safe_r = sr
        # DANGER
        self.danger_r = dr
        # BIG DANGER
        self.big_danger_r = bdr

        # les discretisation pour safe,danger,big_danger
        self.disc = disc

def min_bounds(bounds):
    return bounds.big_danger_l

def max_bounds(bounds):
    return bounds.big_danger_r

def size_bounds(bounds):
    """
        Methode qui retourne la taille en nombre de step de la struct bounds
    """
    bdl = (bounds.danger_l - bounds.big_danger_l) / bounds.disc[2] 
    dl = (bounds.safe_l- bounds.danger_l) / bounds.disc[1] 

    s  = (bounds.safe_r - bounds.safe_l) /bounds.disc[0]

    dr = (bounds.danger_r - bounds.safe_r) / bounds.disc[1] 
    bdr = (bounds.big_danger_r - bounds.danger_r) / bounds.disc[2] 

    return round(bdl+dl+s+dr+bdr) +1 # pour zero 


    # return ceil( (bounds[3]-bounds[0]) /bounds[4]) +1

def center_bounds(bounds):
    b = bounds.big_danger_l
    s = 0

    if bounds.big_danger_l >= 0:
        # TODO 
        print("CAS PAS ENCORE GERER DE DECALAGE VERS LA GAUCHE")
        exit()

    cc = [bounds.danger_l,bounds.safe_l,bounds.safe_r] 

    for i in range(len(cc)):
        r = center_bounds_step(b,cc[i],bounds,2-i)
        s += r[0]
        # print("r0 : {}".format(r[0]))
        if r[1] is None:
            return s
        b = r[1]

    #TODO
    print("ERROR : PQ TU ES ICI ? ")
    exit()

def center_bounds_step(b,b1,bounds,i):
    # print("deb {}".format(b-b1))
    if b1 < 0:
        s = abs(b-b1) / bounds.disc[i]
        # print("dde {}".format(s) )
        return(round(s),b1)
    else:
        s = abs(b-0) / bounds.disc[i]
        return (round(s), None) 


def discretize_bounds(v,bounds):
    """
        Si dans les bornes
            Retourne la valeur et le label
        Sinon
            Retourne None
    """
    # TENTE DES TRUCS
    if (bounds.big_danger_l > v):
        v = bounds.big_danger_l
        # print("EN DEHORS DES BORNES")
        # return None
    if  (v > bounds.big_danger_r):
        v = bounds.big_danger_r
        # print("EN DEHORS DES BORNES")
        # return None

    # si dans big danger l
    if (bounds.big_danger_l <= v  and v < bounds.danger_l ):
        res =  abs(v-bounds.big_danger_l ) / bounds.disc[2]
        return (round(res),LABEL_BIG_DANGER)
    else:
        res = (bounds.danger_l - bounds.big_danger_l) / bounds.disc[2] 

    # si dans danger l
    if (bounds.danger_l <= v  and v < bounds.safe_l) :
        res += abs(v-bounds.danger_l) / bounds.disc[1]
        return (round(res),LABEL_DANGER)
    else:
        res += (bounds.safe_l- bounds.danger_l) / bounds.disc[1] 

    # si dans safe 
    if (bounds.safe_l <= v  and v <= bounds.safe_r ) :
        res += abs(v-bounds.safe_l) / bounds.disc[0]
        return (round(res),LABEL_SAFE)
    else:
        res += (bounds.safe_r - bounds.safe_l) /bounds.disc[0]

    if (bounds.safe_r < v  and v <= bounds.danger_r ) :
        res += abs(v-bounds.safe_r ) / bounds.disc[1]
        return (round(res),LABEL_DANGER)
    else:
        res += (bounds.danger_r - bounds.safe_r) / bounds.disc[1] 

    if (bounds.danger_r < v  and v <= bounds.big_danger_r) :
        res += abs(v-bounds.danger_r ) / bounds.disc[2]
        return (round(res),LABEL_BIG_DANGER)



    print("SHOULD NOT BE HERE")
    print("OUT OF BOUNDS")
    exit()

def get_bounds_array(b):
    e = b.big_danger_l
    res = [e]

    lims = [ b.big_danger_r,b.danger_r,b.safe_r,b.safe_l,b.danger_l]
    il = [2,1,0,1,2]
    
    while len(lims) > 0:
        lim = lims.pop()
        i = il.pop()
        while e < lim:
            e = round(e + b.disc[i],2)
            res.append(e)
    return res

def inbounds(s,b):
    x = s[0]
    y = s[1]

    sx = s[2]
    sy = s[3]
    a = s[4]+s[5]

    if len(b) == 5:
        if x < min_bounds(b[0]) or max_bounds(b[0]) < x:
            return False
        if y < min_bounds(b[1]) or max_bounds(b[1]) < y:
            return False
        if sx < min_bounds(b[2]) or max_bounds(b[2]) < sx:
            return False
        if sy < min_bounds(b[3]) or max_bounds(b[3]) < sy:
            return False
        if a < min_bounds(b[4]) or max_bounds(b[4]) < a:
            return False
        return True
    elif len(b) == 3:
        if sx < min_bounds(b[0]) or max_bounds(b[0]) < sx:
            return False
        if sy < min_bounds(b[1]) or max_bounds(b[1]) < sy:
            return False
        if a < min_bounds(b[2]) or max_bounds(b[2]) < a:
            return False
        return True
    else:
        print("UNKNOW BOUNDS")

############################################################################################
####################################### STABILITY ##########################################
############################################################################################

def discretize_speed_angle(s,bounds):
    """
    Method qui discretise selon les 3 axes 
    speedx
    speedy
    angle 
    """
    bsx = bounds[0]
    bsy = bounds[1]
    ba  = bounds[2]

    a = discretize_bounds(s[4]+s[5], ba)
    b = discretize_bounds(s[2], bsx)
    c = discretize_bounds(s[3], bsy)

    a_size = size_bounds(ba)
    b_size = size_bounds(bsx)
    c_size = size_bounds(bsy)

    # Choisi le label
    label = LABEL_SAFE
    for lab in [a[1],b[1],c[1]]:
        if lab == LABEL_DANGER:
            label = LABEL_DANGER # pas de break car il peut y avoir big danger
        elif lab == LABEL_BIG_DANGER:
            label = LABEL_BIG_DANGER
            break

    res = b[0] + (c[0]*b_size) + (a[0]*b_size*c_size)

    return (res,label)

############################################################################################
########################################## COMPLETE ########################################
############################################################################################
def discretize_complete(s,bounds):
    """
    Method qui discretise selon les 3 axes 
    angle 
    speedx
    speedy
    """
    bpx = bounds[0]
    bpy = bounds[1]
    bsx = bounds[2]
    bsy = bounds[3]
    ba  = bounds[4]

    x = discretize_bounds(s[0], bpx)
    y = discretize_bounds(s[1], bpy)

    a = discretize_bounds(s[4]+s[5], ba)
    b = discretize_bounds(s[2], bsx)
    c = discretize_bounds(s[3], bsy)

    x_size = size_bounds(bpx)
    y_size = size_bounds(bpy)

    a_size = size_bounds(ba)
    b_size = size_bounds(bsx)
    c_size = size_bounds(bsy)

    # Choisi le label
    label = LABEL_SAFE
    for lab in [a[1],b[1],c[1]]: # pas de label si n'est pas safe
        if lab == LABEL_DANGER or lab == LABEL_BIG_DANGER:
            label = None
            break

    res = x[0] + (y[0]*x_size) + (b[0]*x_size*y_size ) + (c[0]*x_size*y_size*b_size) + (a[0]*x_size*y_size*b_size*c_size)

    return (res,label)

    