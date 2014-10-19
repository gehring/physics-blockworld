from Box2D import b2World, b2PolygonShape

class BlockWorld(object):
    def __init__(self,
                 boxes,
                 boxes_density,
                 boxes_restitution,
                 boxes_friction,
                 boxes_pos,
                 boxes_angle,
                 grip_strength,
                 gravity = (0, -9.81),
                 timestep = 1.0/10,
                 vel_iters = 10,
                 pos_iters = 8):

        # init box2d world
        self.world = b2World(gravity = gravity, doSleep = True)

        # gripper max force
        self.grip_strength = grip_strength

        # set simulation parameters
        self.timestep = timestep
        self.vel_iters = vel_iters
        self.pos_iters = pos_iters

        # initialize all the boxes
        self.box_bodies = []
        self.box_fixtures = []
        for b, d, r, f, p, a in zip(boxes,
                               boxes_density,
                               boxes_restitution,
                               boxes_friction,
                               boxes_pos,
                               boxes_angle):
            self.box_bodies.append(self.world.CreateDynamicBody(position = p,
                                                                angle = a))
            fixture = self.box_bodies[-1].CreatePolygonFixture(box=b,
                                                             density = d,
                                                             friction = f,
                                                             restitution = r)
            self.box_fixtures.append(fixture)

        # initialize the floor
        self.floor_body = self.world.CreateStaticBody(
                                    position = (0,0),
                                    shape = b2PolygonShape(box = (50,2))
                                    )

    def step(self, action):
        self.world.Step(self.timestep, self.vel_iters, self.pos_iters)
        self.world.ClearForces()
