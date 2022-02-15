# This is a class contains information and methods with mass.
# parameters:
#               mass: a constants double
#               position: a 3-dimensional vector
#               velocity: a 3-dimensional vector
#               acceleration: a 3-dimensional vector


from vpython import vector


class mass:
    def __init__(self, m: float = 0.1,
                 position=vector(0, 0, 0),
                 velocity=vector(0, 0, 0),
                 acceleration=vector(0, 0, 0),
                 ):
        self.m = m
        self.position = position
        self.velocity = velocity
        self.acceleration = acceleration
        self.force = vector(0, 0, 0)
