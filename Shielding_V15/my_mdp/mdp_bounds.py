from utils_d.discretize import Bounds,size_bounds

def get_stab_bounds():
    # bornes pour le shield de stabilisation 

    bsx = Bounds(   -0.6,
                    -0.3,
                    -0.1,
                    0.1,
                    0.3,
                    0.6,
                    (0.1,0.1,0.1))

    bsy = Bounds(   -0.4,
                    -0.3,
                    -0.1,
                    0.05,
                    0.1,
                    0.2,
                    (0.05,0.05,0.1))

    ba = Bounds(    -0.45,
                    -0.15,
                    -0.05,
                    0.05,
                    0.15,
                    0.45,
                    (0.05,0.05,0.2))
    
    return [bsx,bsy,ba]


def complete_bounds():
    """
        retourne la liste des bornes pour la complete MDP
    """
    # bornes pour le shield d'évitement de la soucoupe
    bpx = Bounds(   -0.5,
                    -0.3,
                    -0.1,
                    0.1,
                    0.3,
                    0.5,
                    (0.1,0.1,0.1))

    # borne a l'envers par rapport au referenciel 1.2 HAUT 
    bpy = Bounds(   0,
                    0.2,
                    0.3,
                    0.5,
                    1.0,
                    1.2,
                    (0.1,0.1,0.1))

    ba = Bounds(    -0.15,
                    -0.15,
                    -0.05,
                    0.05,
                    0.15,
                    0.15,
                    (0.05,0.05,0.2))

    bsx = Bounds(   -0.2,
                    -0.2,
                    -0.1,
                    0.1,
                    0.2,
                    0.2,
                    (0.1,0.1,0.1))

    bsy = Bounds(   -0.3,
                    -0.3,
                    -0.1,
                    0.05,
                    0.1,
                    0.2,
                    (0.05,0.05,0.1))

    return[bpx,bpy,bsx,bsy,ba]

def hit_bounds():
    """
        retourne la liste des bornes pour la HIT MDP
    """
    # bornes pour le shield d'évitement de la soucoupe
    bpx = Bounds(   -0.3,
                    -0.3,
                    -0.3,
                    0.3,
                    0.3,
                    0.3,
                    (0.1,0.1,0.1))

    # borne a l'envers par rapport au referenciel 1.2 HAUT 
    bpy = Bounds(   0.6,
                    0.6,
                    0.6,
                    1.0,
                    1.0,
                    1.0,
                    (0.1,0.1,0.1))

    stab = get_stab_bounds()

    return[bpx,bpy,stab[0],stab[1],stab[2]]

def printsize():
    b1 = get_stab_bounds()
    b2 = complete_bounds()
    b3 = hit_bounds()

    for b in [b1,b2,b3]:
        size = 1
        for e in b:
            size *= size_bounds(e)
        print(size)
