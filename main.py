import concurrent.futures
import math
import random as rand

from vpython import *
import Monitor
import time
import MyRobot
import csv


GRAVITY = vector(0, 0, -9.81)
DAMPING = 0.9992
DT = 0.0001
FRICTION = 0.7
K_VERTICES_SOFT = 1000.0
K_GROUND = 100000.0
OMEGA = 10.0


'''
physic simulator & update function
'''


def update_robot(r, g, t):
    for m in r.mass_list:
        m.force = vector(0, 0, -9.81*m.m)

    for s in r.spring_list:
        index = int(r.spring_list.index(s))
        m1 = r.mass_list[s.m1]
        m2 = r.mass_list[s.m2]
        l_0_new = s.l_0 + g[index][1] * s.l_0 * math.sin(2*math.pi*t + g[index][2])
        spring_force = g[index][0] * (mag(m1.position - m2.position) - l_0_new)
        m1.force = m1.force + (spring_force / mag(m1.position - m2.position)) * (m2.position - m1.position)
        m2.force = m2.force + (spring_force / mag(m1.position - m2.position)) * (m1.position - m2.position)

    for m in r.mass_list:
        if m.position.z < 0:  # check if a point hits the ground.
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


'''
physic simulator & update function finished.
'''


'''
Genetic Algorithm
'''
POPULATION_SIZE = 24


def generate_population():
    p = []
    gene = []
    for i in range(POPULATION_SIZE):
        p.append(MyRobot.Robot())
        gene.append(generate_genes())
    return p, gene


def generate_genes(robot):
    genes = []
    for i in range(0, len(robot.spring_list)):
        genes.append([rand.randrange(1000, 10000, 1000), rand.uniform(-0.2, 0.2), rand.uniform(0, 2*math.pi)])
        # randomly generate k, b & c in l_0 = a + b*sin(wt+c) and k
    return genes


def mutation(individual):  # individual structure is a list[robot, gene_list, distance]
    mutation_rate = 0.1
    offspring = individual[1]  # get the gene_list of that individual
    for childID in range(0, len(offspring)):  # for every k, b, c, mutate individually.
        mutation_probability = rand.random()
        if mutation_probability < mutation_rate:  # mutate k
            offspring[childID][0] = rand.randrange(1000, 10000, 1000)
        mutation_probability = rand.random()
        if mutation_probability < mutation_rate:  # mutate b
            offspring[childID][1] = rand.uniform(0, 0.1)
        mutation_probability = rand.random()
        if mutation_probability < mutation_rate:  # mutate c
            offspring[childID][2] = rand.uniform(0, 2*math.pi)


def cross(f, m):
    '''
    to crossover father and mother and return 2 new offsprings.
    :param f: father individual
    :param m: mother individual
    :return: 2 new offsprings.
    '''
    cross_point1 = rand.randint(1, int(len(f[1])-1))
    cross_point2 = rand.randint(1, int(len(m[1])-1))
    if cross_point1 > cross_point2:
        cross_point1, cross_point2 = cross_point2, cross_point1
    for cross_point in range(cross_point1, cross_point2+1):
        f[1][cross_point], m[1][cross_point] = m[1][cross_point], f[1][cross_point]
    mutation(f)
    mutation(m)
    #print(cross_point1)
    #print(cross_point2)
    return [f, m]


def crossover(p):
    '''
    :param p: a list of parents.
    :return: a list of parents and offsprings produced by crossover and mutation.
    '''

    father = p[0:int(len(p)):2]
    mother = p[int(len(p)):0:-2]
    '''for i in range(0,len(father)):
        print(father[i][1])
        print(mother[i][1])
        print('---------------------------------------')
    print('+++++++++++++++++++++++++++++++++++++++++++++++++++')'''
    with concurrent.futures.ProcessPoolExecutor() as executor:
        for offsprings in executor.map(cross, father, mother):
            p.append(offsprings[0])
            p.append(offsprings[1])
            '''print(offsprings[0][1])
            print(offsprings[1][1])
            print('---------------------------------------')'''
    return p


def selection(p):  # select top 50% to crossover, and put them into next generation.
    return p[int(len(p)-1): int(len(p)/2-1): -1]


def distance(individual):  # calculate an individual moving distance in 5 seconds.
    t = 0
    r = individual[0]
    g = individual[1]
    while t < 5:
        # rate(0.8/DT)
        update_robot(r, g, t)
        # for individual in range(0, POPULATION_SIZE):
        #    update_robot(population[individual], gene_list[individual], t)
        # Monitor.update(r.mass_list, r.spring_list)
        # print(r.mass_list[0].position)
        t = t + DT
    com = COM(r)
    return mag(vector(com.x, com.y, 0))


def population_distance(a_population):
    # start_time = time.time()
    i = 0
    with concurrent.futures.ProcessPoolExecutor() as executor:
        for Rid, dist in zip(list(range(1, POPULATION_SIZE+1)),
                             executor.map(distance, a_population)):
            print('%d robot moving distance: %f' % (Rid, dist))
            a_population[i][2] = dist
            i += 1
    # print(time.time() - start_time)


def GA(iteration):
    p, gene_list = generate_population()
    d = [0.0] * POPULATION_SIZE
    population = list(map(list, zip(p, gene_list, d)))
    # an individual include a robot, a gene_list and a distance value.
    '''all individual information are stored in a list[robot, gene, distance]'''

    dot_plot_file = open('dotplot.csv', 'w', newline='')
    learning_curve_file = open('learningcurve.csv', 'w', newline='')
    dot_writer = csv.writer(dot_plot_file)
    learning_writer = csv.writer(learning_curve_file)
    dot_writer.writerow(['generation', 'distance'])
    learning_writer.writerow(['generation', 'distance'])

    max_gen = iteration  # we evolve the population in 1000 generations.
    for i in range(1, max_gen+1):
        start_time = time.time()
        population_distance(population)
        population.sort(key=lambda dis: dis[2])  # individuals are already sorted here, we can directly select top 50%.
        print('generation %d: best robot moves %f m' % (i, population[POPULATION_SIZE-1][2]))
        learning_writer.writerow([str(i), str(population[POPULATION_SIZE-1][2])])
        gene_info_file = open('gene.txt', 'w')
        for individual in population:
            dot_writer.writerow([str(i), str(individual[2])])
            gene_info_file.write(str(individual[1]) + '\n')
        gene_info_file.close()
        parents = selection(population)
        population = crossover(parents)
        print('calculation time: '+str(time.time() - start_time)+' s')
    dot_plot_file.close()
    learning_curve_file.close()
    return population[POPULATION_SIZE-1][1]


def COM(r):  # calculate coordinates of the center of mass of the robot.
    com = vector(0, 0, 0)
    total_mass = 0.0
    for mass_point in r.mass_list:
        com += mass_point.position * mass_point.m
        total_mass += mass_point.m
    com = com / total_mass
    return com


'''
Genetic Algorithm finished
'''


'''
Display and Monitor Functions
'''


def animate(r, g):
    Monitor.display(r.mass_list, r.spring_list)
    scene = canvas.get_selected()
    scene.waitfor('mouseup')
    t = 0
    while t < 5:
        rate(1/DT)
        update_robot(r, g, t)
        Monitor.update(r.mass_list, r.spring_list)
        # print(r.mass_list[0].position)
        t = t + DT


'''
Display and Monitor Functions finished
'''


if __name__ == '__main__':
    Monitor.figure()
    r1 = MyRobot.Robot()
    r2 = MyRobot.Robot(startpoint=vector(0.2,0,0))
    r2.addPoint(vector(0.23,0.15,0.06), 0, 1, 2)
    r3 = MyRobot.Robot(startpoint=vector(-0.2, 0, 0))
    r3.addPoint(vector(-0.1, -0.08, 0.06), 0, 1, 2)
    r3.addPoint(vector(-0.13, -0.15, 0.08), 3, 4, 5)
    g1 = generate_genes(r1)
    g2 = generate_genes(r2)
    g3 = generate_genes(r3)
    p1, l1 = Monitor.display(r1.mass_list, r1.spring_list)
    p2, l2 = Monitor.display(r2.mass_list, r2.spring_list)
    p3, l3 = Monitor.display(r3.mass_list, r3.spring_list)
    scene = canvas.get_selected()
    scene.waitfor('mouseup')
    t = 0
    while t < 5:
        # rate(1/DT)
        update_robot(r1, g1, t)
        update_robot(r2, g2, t)
        update_robot(r3, g3, t)
        Monitor.update(r1.mass_list, r1.spring_list, p1, l1)
        Monitor.update(r2.mass_list, r2.spring_list, p2, l2)
        Monitor.update(r3.mass_list, r3.spring_list, p3, l3)
        # print(r.mass_list[0].position)
        t = t + DT