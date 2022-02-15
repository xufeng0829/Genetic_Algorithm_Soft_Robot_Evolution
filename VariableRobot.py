import concurrent.futures
import copy
import time
import pandas as pd
from vpython import *
import mass
import spring
import numpy as np
import random as rand
import Monitor
import math

DIMENSION_SCALE = 0.1
GRAVITY = vector(0, 0, -9.81)
DAMPING = 0.9992
DT = 0.001
FRICTION = 0.8
K_VERTICES_SOFT = 1000.0
K_GROUND = 25000.0
OMEGA = 10.0
materials = {1: [0, 0, 0], 2: [1000, 0, 0], 3: [5000, 0, 0], 4: [1000, 0.2, 0.5 * pi]}
POPULATION_SIZE = 10


# materials are 4 types:
#           0: air, no spring
#           1: soft tissue
#           2: hard structure
#           3: breathing tissue


class Robot:
    '''
        every robot contains a list of mass points, a list of spring points, a list of existed point, and a list of
    existed mass point pairs of springs.
    '''

    def __init__(self):
        self.mass_list = []
        self.spring_list = []
        self.exist_mass = []
        self.exist_spring = []

    def build_robot(self, gene):
        self.mass_list = []
        self.spring_list = []
        self.exist_mass = []
        self.exist_spring = []
        # numpy 3-d array coordinate is z, x, y
        for x in range(0, 3):
            for y in range(0, 3):
                for z in range(0, 3):
                    if gene[z, x, y] == 1:
                        pass
                    else:
                        self.add_mass(x, y, z)
        for x in range(0, 3):
            for y in range(0, 3):
                for z in range(0, 3):
                    if gene[z, x, y] == 1:
                        pass
                    else:
                        material = materials.get(gene[z, x, y])
                        self.add_spring(x, y, z, material)

    def point(self, x, y, z) -> int:
        '''

        this function can help find the index of a mass point in mass_list.
        '''
        for i in range(0, len(self.mass_list)):
            if self.mass_list[i].position == vector(x, y, z)*DIMENSION_SCALE:
                return i
        return 0

    def add_mass(self, x, y, z):
        '''
        The cube looks like:
             z
             |
            4------------6
           /|           /|
          5-|----------7 |
          | |          | |
          | 0----------+-2-------->y
          |/           |/
          1------------3
         /
        x
        '''
        for x_0 in range(x, x + 2):
            for y_0 in range(y, y + 2):
                for z_0 in range(z, z + 2):
                    if vector(x_0, y_0, z_0) * DIMENSION_SCALE not in self.exist_mass:
                        # if this mass point is a new point, add it to mass_list
                        self.mass_list.append(mass.mass(m=0.1, position=vector(x_0, y_0, z_0) * DIMENSION_SCALE))
                        self.exist_mass.append(vector(x_0, y_0, z_0) * DIMENSION_SCALE)
                    else:
                        pass

    def add_spring(self, x, y, z, material):
        for x_0 in range(x, x + 2):
            for y_0 in range(y, y + 2):
                for z_0 in range(z, z + 2):
                    for x_1 in range(x, x + 2):
                        for y_1 in range(y, y + 2):
                            for z_1 in range(z, z + 2):
                                if (vector(x_0, y_0, z_0) * DIMENSION_SCALE, vector(x_1, y_1, z_1) * DIMENSION_SCALE) \
                                        not in self.exist_spring and vector(x_0, y_0, z_0) != vector(x_1, y_1, z_1):
                                    # if this spring is a new spring, add it to spring list
                                    m1 = self.point(x_0, y_0, z_0)
                                    m2 = self.point(x_1, y_1, z_1)
                                    self.spring_list.append(
                                        spring.spring(mag(self.mass_list[m1].position - self.mass_list[m2].position),
                                                      m1, m2, material))
                                    self.exist_spring.append(
                                        (vector(x_0, y_0, z_0) * DIMENSION_SCALE,
                                         vector(x_1, y_1, z_1) * DIMENSION_SCALE))
                                # print((x_0,y_0,z_0), (x_1, y_1, z_1))


def update_robot(r, t):
    for m in r.mass_list:
        if t > 0:
            m.force = vector(0, 0, -9.81*m.m)
        else:
            m.force = vector(0, 0, 0)

    for s in r.spring_list:
        m1 = r.mass_list[s.m1]
        m2 = r.mass_list[s.m2]
        k = s.parameter[0]
        b = s.parameter[1]
        c = s.parameter[2]
        l_0_new = s.l_0 + b * s.l_0 * math.sin(2*pi*t + c)
        spring_force = k * (mag(m1.position - m2.position) - l_0_new)
        m1.force = m1.force + spring_force * norm(m2.position - m1.position)
        m2.force = m2.force + spring_force * norm(m1.position - m2.position)

    for m in r.mass_list:
        if m.position.z <= 0:  # check if a point hits the ground.
            horizon_force = vector(m.force.x, m.force.y, 0)
            m.force = m.force + vector(0, 0, K_GROUND * (0 - m.position.z))
            if mag(horizon_force) <= FRICTION * K_GROUND * (0 - m.position.z):  # calculate friction force
                m.force.x = 0
                m.force.y = 0
            else:
                m.force = m.force - FRICTION * K_GROUND * (0 - m.position.z) * norm(horizon_force)
        m.acceleration = m.force / m.m
        m.velocity = m.velocity + m.acceleration * DT
        m.velocity = m.velocity * DAMPING
        m.position = m.position + m.velocity * DT


def build_gene():
    """
    :return: a 3*3*3 matrix contains random types of cubes in a robot
    """
    return np.random.randint(1, 5, (3, 3, 3))


def COM(r):  # calculate coordinates of the center of mass of the robot.
    com = vector(0, 0, 0)
    total_mass = 0.0
    for mass_point in r.mass_list:
        com += mass_point.position * mass_point.m
        total_mass += mass_point.m
    com = com / total_mass
    return com


def build_individual():
    """
    to build an individual with random gene.
    :return: a list contains all individual's information.
                [robot instance, gene matrix, fitness value(set 0 at default.)]
    """
    robot = Robot()
    gene = build_gene()
    fitness = 0.0
    individual = [robot, gene, fitness]
    return individual


def build_population():
    """
    This function can build a population with <POPULATION_SIZE> individuals.
    :return: a list of individuals.
    """
    population = []
    for i in range(0, POPULATION_SIZE):
        population.append(build_individual())
    return population


def distance(individual):
    """
    This function calculate an individual's fitness, which is how far it can move.
    :param individual:
    :return: the moving distance in 3 seconds.
    """
    t = 0
    r = individual[0]
    g = individual[1]
    r.build_robot(g)
    com_start = COM(r)
    while t < 3:
        update_robot(r, t)
        t = t + DT
    com_end = COM(r)
    return mag(vector(com_end.x, com_end.y, 0) - vector(com_start.x, com_start.y, 0))


def calculate_population_distance(a_population):
    i = 0
    print('calculating fitness')
    with concurrent.futures.ProcessPoolExecutor(max_workers=10) as executor:
        for Rid, dist in zip(list(range(1, POPULATION_SIZE + 1)),
                             executor.map(distance, a_population)):
            # print('    %d robot moving distance: %f' % (Rid, dist))
            # print('.', end='')
            a_population[i][2] = dist
            i += 1
    # print('done.')


def mutation(individual):
    x_mute = rand.randint(0, 2)
    y_mute = rand.randint(0, 2)
    z_mute = rand.randint(0, 2)
    value = rand.randint(1, 4)
    # print(x_mute, y_mute, z_mute)
    # print('from: ' + str(individual[1][z_mute, x_mute, y_mute]) + ' to: ' + str(value))
    individual[1][z_mute, x_mute, y_mute] = value


def tournament_selection(population):
    """
    This is a 2 individual competition tournament selection.
    :param population: a population waiting for selecting parents into next generation.
    :return: a population of parents.
    """
    parents = []
    for tournament_round in range(0, len(population)):
        parent_index1 = rand.randint(0, POPULATION_SIZE - 1)  # randint is a closed interval. need POPULATION_SIZE-1.
        parent_index2 = rand.randint(0, POPULATION_SIZE - 1)
        # print(parent_index1, parent_index2, end=': ')
        if population[parent_index1][2] >= population[parent_index2][2]:
            parents.append(population[parent_index1])
            # print('index: ' + str(parent_index1), end=', ')
        else:
            parents.append(population[parent_index2])
            # print('index: ' + str(parent_index2), end=', ')
        # print('')
    # print('')
    return parents


def crossover(parents):
    """
    crossover two parents to generate 2 new individuals. Then, mutate them after crossover.
    :param parents: population of parents
    :return: a population of offsprings
    """
    offsprings = []
    for crossover_round in range(0, len(parents), 2):
        father = parents[crossover_round]
        mother = parents[crossover_round + 1]
        crossover_matrix1 = np.random.randint(0, 2, (3, 3, 3))
        crossover_matrix2 = np.ones((3, 3, 3), dtype=int) - crossover_matrix1
        offspring1 = copy.deepcopy(father)
        offspring2 = copy.deepcopy(mother)
        offspring1[1] = father[1] * crossover_matrix1 + mother[1] * crossover_matrix2
        offspring2[1] = father[1] * crossover_matrix2 + mother[1] * crossover_matrix1
        # mutation rate is 0.1 here.
        # if rand.random() < 0.1:
        mutation(offspring1)
        # if rand.random() < 0.1:
        mutation(offspring2)
        offsprings.append(offspring1)
        offsprings.append(offspring2)
    return offsprings


def crossover_alter(parents):
    """
    crossover two parents to generate 2 new individuals by swapping sub-cubes. Then, mutate them after crossover.
    :param parents: population of parents
    :return: a population of offsprings
    """
    offsprings = []
    for crossover_round in range(0, len(parents), 2):
        father = parents[crossover_round]
        mother = parents[crossover_round + 1]
        x1 = rand.randint(0, 1)
        y1 = rand.randint(0, 1)
        z1 = rand.randint(0, 1)
        x2 = rand.randint(0, 1)
        y2 = rand.randint(0, 1)
        z2 = rand.randint(0, 1)
        offspring1 = copy.deepcopy(father)
        offspring2 = copy.deepcopy(mother)
        for i in range(0, 2):
            offspring1[1][z1 + i][x1 + i][y1 + i] = mother[1][z2 + i][x2 + i][y2 + i]
            offspring2[1][z2 + i][x2 + i][y2 + i] = father[1][z1 + i][x1 + i][y1 + i]
        # if rand.random() < 0.1:
        mutation(offspring1)
        # if rand.random() < 0.1:
        mutation(offspring2)
        offsprings.append(offspring1)
        offsprings.append(offspring2)
    return offsprings


def difference(individual1, individual2):
    """
    This is a function to calculate the difference between 2 individuals in genotypes.
    :param individual1:
    :param individual2:
    :return: an value of differences.
    """
    gene1 = individual1[1]
    gene2 = individual2[1]
    diff_vector = vector(0, 0, 0)
    for x in range(0, 3):
        for y in range(0, 3):
            for z in range(0, 3):
                if gene1[x, y, z] == gene2[x, y, z]:
                    pass
                else:
                    diff_vector += vector(x, y, z)
    return mag(diff_vector)


def competition(parents, offsprings):
    """
    This is a function to compete parents and offsprings to build next generation, by using DETERMINISTIC CROWDING to
    maintain diversity.
    :param parents: population of parents
    :param offsprings: population of offsprings
    :return: population of next generation
    """
    # parents are just selected, no change on gene, so no need to calculate their distance.
    # offsprings genes are changed. Need to calculate distance first before sending them into this function!

    next_gen = []
    for competition_round in range(0, len(parents), 2):
        parent1 = parents[competition_round]
        parent2 = parents[competition_round + 1]
        child1 = offsprings[competition_round]
        child2 = offsprings[competition_round + 1]
        if difference(parent1, child1)+difference(parent2, child2)\
                < difference(parent1, child2)+difference(parent2, child1):
            if child1[2] > parent1[2]:
                next_gen.append(child1)
                # print('better!', end=' ')
            else:
                next_gen.append(parent1)
            if child2[2] > parent2[2]:
                next_gen.append(child2)
                # print('better!', end=' ')
            else:
                next_gen.append(parent2)
        else:
            if child1[2] > parent2[2]:
                next_gen.append(child1)
                # print('better!', end=' ')
            else:
                next_gen.append(parent2)
            if child2[2] > parent1[2]:
                next_gen.append(child2)
                # print('better!', end=' ')
            else:
                next_gen.append(parent1)
    return next_gen


def key(individual):
    return individual[2]


'''
Here is different kinds of algorithms.
'''


def random_search(generation):
    gen = 1
    result = [[], []]
    best_dist = 0
    while gen <= generation:
        print('generation ' + str(gen))
        start = time.time()
        indi = build_individual()
        dist = distance(indi)
        if best_dist < dist:
            best_dist = dist
        result[0].append(gen)
        result[1].append(best_dist)
        print('process time: ' + str(time.time() - start))
        gen += 1

    df = pd.DataFrame(result)
    df = df.T
    df.to_csv('rs.csv', index=0)


def hill_climbing(generation):
    gen = 1
    result = [[], []]
    best_dist = 0
    best_gene = []
    indi = build_individual()
    while gen <= generation:
        print('generation ' + str(gen))
        start = time.time()
        dist = distance(indi)
        if best_dist < dist:
            best_dist = dist
            best_gene = indi[1]
        else:
            indi[1] = best_gene
        result[0].append(gen)
        result[1].append(best_dist)
        mutation(indi)
        print('process time: ' + str(time.time() - start))
        gen += 1

    df = pd.DataFrame(result)
    df = df.T
    df.to_csv('hc.csv', index=0)


def GA(generation):
    gen = 1
    result = [[], []]
    best_dist = 0
    best_gene = []
    print('initializing...')
    p = build_population()
    calculate_population_distance(p)
    print('initialization done.')
    while gen <= generation:
        print('generation ' + str(gen))
        start = time.time()
        p.sort(key=key, reverse=True)  # need to sort from highest to lowest.
        if best_dist < p[0][2]:
            best_dist = p[0][2]
            best_gene = p[0][1]
        # parents are just selected, no change on gene, so no need to calculate their distance.
        parents = tournament_selection(p)
        # offsprings genes are changed.
        offsprings = crossover_alter(parents)
        calculate_population_distance(offsprings)
        p = competition(parents, offsprings)
        for i in p:
            print('distance: ' + str(i[2]))
        result[0].append(gen)
        result[1].append(best_dist)
        print('best distance: ' + str(best_dist) + 'm', end=', ')
        print('process time: ' + str(time.time() - start))
        gen += 1

    print(best_gene)
    df = pd.DataFrame(result)
    df = df.T
    df.to_csv('ga.csv', index=0)


if __name__ == '__main__':
    GA(100)
