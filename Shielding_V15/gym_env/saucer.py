from gym_env.lunar_lander import (LunarLander,
                          SCALE,
                          VIEWPORT_W,
                          VIEWPORT_H,
                          ContactDetector)

from Box2D.b2 import fixtureDef, polygonShape, contactListener


H = VIEWPORT_H/SCALE
W = VIEWPORT_W/SCALE
SAUCER_POLY = [
    (-20, +6),
    (-17, +12),
    (+17, +12),
    (+20, +6),
    (+17, 0),
    (-17, 0),
]

WINDOW_POLY = [
    (-16, +10),
    (-12, +16),
    (+12, +16),
    (+16, +10),
]

class ContactDetectorWithSaucer(ContactDetector):
    def BeginContact(self, contact):
        super().BeginContact(contact)

        for i in [0,1]:
            if self.env.saucer in [contact.fixtureA.body, contact.fixtureB.body] and self.env.legs[i] in [contact.fixtureA.body, contact.fixtureB.body]:
                self.env.hitSaucer = True

            if self.env.window in [contact.fixtureA.body, contact.fixtureB.body] and self.env.legs[i] in [contact.fixtureA.body, contact.fixtureB.body]:
                self.env.hitSaucer = True
        
    def EndContact(self, contact):
        super().EndContact(contact)
        for i in [0,1]:
            if self.env.saucer in [contact.fixtureA.body, contact.fixtureB.body] and self.env.legs[i] in [contact.fixtureA.body, contact.fixtureB.body]:
                self.hitSaucer = False

            if self.env.window in [contact.fixtureA.body, contact.fixtureB.body] and self.env.legs[i] in [contact.fixtureA.body, contact.fixtureB.body]:
                self.hitSaucer = False

class LanderObstacle(LunarLander):
    """
    Subclass of Gym's LunarLander.
    Has an obstacle (a saucer) beneath the starting position of the lander.
    """
    def __init__(self, stretch_x=1., stretch_y=1., y_translation=0.):
        """
        Class constructor

        Parameters:
            stretch_x: stretches the saucer horizontally
            stretch_y: streches the saucer vertically
            y_translation: moves the saucer vertically
        """
        self.saucer = None
        self.window = None
        self.hitSaucer = False
        y = H/2
        x = W/2
        self.saucer_fixtures = fixtureDef(
                shape=polygonShape(vertices=[
                    (stretch_x * p[0]/SCALE + x,
                     stretch_y * p[1]/SCALE + y + y_translation)
                    for p in SAUCER_POLY
                ],)
        )

        self.window_fixtures = fixtureDef(
                shape=polygonShape(vertices=[
                    (stretch_x * p[0]/SCALE + x,
                     stretch_y * p[1]/SCALE + y + y_translation)
                    for p in WINDOW_POLY
                ],)
            )
        LunarLander.__init__(self)


    def reset(self):
        self.hitSaucer = False
        observation = LunarLander.reset(self)
        self.saucer = self.world.CreateStaticBody(
            shapes=self.moon_shape
        )
        self.window = self.world.CreateStaticBody(
            shapes=self.moon_shape
        )
        self.saucer.CreateFixture(self.saucer_fixtures,
                                  density=0,
                                  friction=0.1)
        self.window.CreateFixture(self.window_fixtures,
                                  density=0,
                                  friction=0.1)
        self.saucer.color1 = (204/255, 51/255, 255/255)
        self.saucer.color2 = (102/255, 0., 102/255)
        self.window.color1 = (51/255, 204/255., 1.)
        self.window.color2 = (0., 1., 1.)
        self.drawlist += [self.saucer, self.window]

        # Set contact listener with saucer
        self.world.contactListener_keepref = ContactDetectorWithSaucer(self)
        self.world.contactListener = self.world.contactListener_keepref
        return observation

    def _destroy(self):
        LunarLander._destroy(self)
        if self.saucer is not None:
            self.world.DestroyBody(self.saucer)
        if self.window is not None:
            self.world.DestroyBody(self.window)

