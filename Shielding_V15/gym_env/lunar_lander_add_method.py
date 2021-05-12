import sys
sys.path.append('..')

import numpy as np

from gym_env.lunar_lander import (VIEWPORT_W,VIEWPORT_H,SCALE,INITIAL_RANDOM,LEG_AWAY,LEG_DOWN,LEG_SPRING_TORQUE)
from gym_env.lunar_lander import ContactDetector
from gym_env.saucer import ContactDetectorWithSaucer
from Box2D.b2 import (edgeShape, circleShape, fixtureDef, polygonShape, revoluteJointDef, contactListener)
from Box2D.Box2D import b2Vec2

def reset_to_pos_saucer(env,posx,posy,angle,speedx,speedy,angular_vel):
    """
    Fonction qui permet de positionner le lander a une position,angle precise
    dans el contexte du saucer
    """
    env.hitSaucer = False
    # appel le reset to pos de l'environment normal
    observation = reset_to_pos(env,posx,posy,angle,speedx,speedy,angular_vel) 
    env.saucer = env.world.CreateStaticBody(
        shapes=env.moon_shape
    )
    env.window = env.world.CreateStaticBody(
        shapes=env.moon_shape
    )
    env.saucer.CreateFixture(env.saucer_fixtures,
                                density=0,
                                friction=0.1)
    env.window.CreateFixture(env.window_fixtures,
                                density=0,
                                friction=0.1)
    env.saucer.color1 = (204/255, 51/255, 255/255)
    env.saucer.color2 = (102/255, 0., 102/255)
    env.window.color1 = (51/255, 204/255., 1.)
    env.window.color2 = (0., 1., 1.)
    env.drawlist += [env.saucer, env.window]
    # Set contact listener with saucer
    env.world.contactListener_keepref = ContactDetectorWithSaucer(env)
    env.world.contactListener = env.world.contactListener_keepref
    return observation

def reset_to_pos(env,posx,posy,angle,speedx,speedy,angular_vel):
    """
    Fonction qui permet de positionner le lander a une position,angle precise
    """
    env._destroy()
    env.world.contactListener_keepref = ContactDetector(env)
    env.world.contactListener = env.world.contactListener_keepref
    env.game_over = False
    env.prev_shaping = None

    W = VIEWPORT_W/SCALE
    H = VIEWPORT_H/SCALE

    # terrain
    CHUNKS = 11
    height = env.np_random.uniform(0, H/2, size=(CHUNKS+1,) )
    chunk_x  = [W/(CHUNKS-1)*i for i in range(CHUNKS)]
    env.helipad_x1 = chunk_x[CHUNKS//2-1]
    env.helipad_x2 = chunk_x[CHUNKS//2+1]
    env.helipad_y  = H/4
    height[CHUNKS//2-2] = env.helipad_y
    height[CHUNKS//2-1] = env.helipad_y
    height[CHUNKS//2+0] = env.helipad_y
    height[CHUNKS//2+1] = env.helipad_y
    height[CHUNKS//2+2] = env.helipad_y
    smooth_y = [0.33*(height[i-1] + height[i+0] + height[i+1]) for i in range(CHUNKS)]

    env.moon = env.world.CreateStaticBody( shapes= env.moon_shape )
    env.sky_polys = []
    for i in range(CHUNKS-1):
        p1 = (chunk_x[i],   smooth_y[i])
        p2 = (chunk_x[i+1], smooth_y[i+1])
        env.moon_fixtures[i].shape.vertices = [p1,p2]
        env.moon.CreateFixture(env.moon_fixtures[i],
                                density=0,
                                friction=0.1)
        env.sky_polys.append( [p1, p2, (p2[0],H), (p1[0],H)] )

    env.moon.color1 = (0.0,0.0,0.0)
    env.moon.color2 = (0.0,0.0,0.0)

    initial_y = VIEWPORT_H/SCALE
    env.lander = env.world.CreateDynamicBody(
        # position = (VIEWPORT_W/SCALE/2, initial_y),
        position = (posx, posy),
        # angle=0.0,
        angle = angle,
        fixtures = env.lander_fixtures
    )
    env.lander.color1 = (0.5,0.4,0.9)
    env.lander.color2 = (0.3,0.3,0.5)
    env.lander.ApplyForceToCenter( (
        env.np_random.uniform(-INITIAL_RANDOM, INITIAL_RANDOM),
        env.np_random.uniform(-INITIAL_RANDOM, INITIAL_RANDOM)
        ), True)
    # env.lander.ApplyForceToCenter( (speedx,speedy), True)

    env.legs = []
    for i in [-1,+1]:
        leg = env.world.CreateDynamicBody(
            # position = (VIEWPORT_W/SCALE/2 - i*LEG_AWAY/SCALE, initial_y),
            position = (posx - i*LEG_AWAY/SCALE, posy),
            # angle = (i*0.05),
            angle = angle + (i*0.05),
            fixtures = env.leg_fixtures
            )
        leg.ground_contact = False
        leg.color1 = (0.5,0.4,0.9)
        leg.color2 = (0.3,0.3,0.5)
        rjd = revoluteJointDef(
            bodyA=env.lander,
            bodyB=leg,
            localAnchorA=(0, 0),
            localAnchorB=(i*LEG_AWAY/SCALE, LEG_DOWN/SCALE),
            enableMotor=True,
            enableLimit=True,
            maxMotorTorque=LEG_SPRING_TORQUE,
            motorSpeed=+0.3*i  # low enough not to jump back into the sky
            )
        if i==-1:
            rjd.lowerAngle = +0.9 - 0.5  # Yes, the most esoteric numbers here, angles legs have freedom to travel within
            rjd.upperAngle = +0.9
        else:
            rjd.lowerAngle = -0.9
            rjd.upperAngle = -0.9 + 0.5
        leg.joint = env.world.CreateJoint(rjd)
        env.legs.append(leg)

    env.drawlist = [env.lander] + env.legs

    ## SPEED 
    env.lander.linearVelocity = b2Vec2(speedx,speedy)

    ## angular velocity
    env.lander.angularVelocity = angular_vel

    return env.step(np.array([0,0]) if env.continuous else 0)[0]