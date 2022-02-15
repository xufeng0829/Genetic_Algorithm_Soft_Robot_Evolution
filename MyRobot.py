from vpython import *
import mass
import spring


GRAVITY = vector(0, 0, -9.81)
DAMPING = 0.9999
DT = 0.0001
FRICTION = 0.7
K_VERTICES_SOFT = 1000.0
K_GROUND = 100000.0
OMEGA = 10.0


class Robot:
    def __init__(self, startpoint=vector(0, 0, 0)):
        self.mass_list = []
        self.spring_list = []
        # p = vector(0, 0, 0)  # moving
        p = startpoint  # bouncing
        length = 0.1
        self.mass_list.append(mass.mass(m=0.1, position=vector(p.x, p.y, p.z)))
        self.mass_list.append(mass.mass(m=0.1, position=vector(p.x + length, p.y, p.z)))
        self.mass_list.append(mass.mass(m=0.1, position=vector(p.x + length / 2, p.y + (sqrt(3) / 2) * length, p.z)))  # 2
        self.mass_list.append(mass.mass(m=0.1, position=vector(p.x + length / 2, p.y + length / (2 * sqrt(3)),
                                                          p.z + length * sqrt(2 / 3))))  # 3
        self.mass_list.append(mass.mass(m=0.1, position=vector(p.x + length / 2, p.y - (sqrt(3) / 2) * length, p.z)))

        diag_1 = length
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, diag_1, 0, 1))  # 0
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, diag_1, 0, 2))  # 1
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, diag_1, 0, 3))  # 2
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, diag_1, 1, 2))  # 3
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, diag_1, 1, 3))  # 4
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, diag_1, 2, 3))  # 5

        self.spring_list.append(spring.spring(K_VERTICES_SOFT, diag_1, 0, 4))  # 6
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, diag_1, 1, 4))  # 7

        self.spring_list.append(spring.spring(K_VERTICES_SOFT, mag(self.mass_list[3].position - self.mass_list[4].position), 3, 4))  # 8

    def addPoint(self, position, m1, m2, m3):
        self.mass_list.append(mass.mass(m=0.1, position=position))

        self.spring_list.append(spring.spring(K_VERTICES_SOFT, mag(
            self.mass_list[m1].position - self.mass_list[len(self.mass_list) - 1].position),
                                              m1, len(self.mass_list) - 1))
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, mag(
            self.mass_list[m2].position - self.mass_list[len(self.mass_list) - 1].position),
                                              m2, len(self.mass_list) - 1))
        self.spring_list.append(spring.spring(K_VERTICES_SOFT, mag(
            self.mass_list[m3].position - self.mass_list[len(self.mass_list) - 1].position),
                                              m3, len(self.mass_list) - 1))
