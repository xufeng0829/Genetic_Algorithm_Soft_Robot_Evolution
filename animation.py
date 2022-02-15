import concurrent.futures
import math
import random as rand
import copy
from vpython import *
import Monitor
import time
import MyRobot
import csv
import main
import VariableRobot
import numpy as np

DT = 0.001


def animate(r):
    Monitor.figure()
    p, l = Monitor.display(r.mass_list, r.spring_list)
    t = 0
    results = []
    i = 0
    print('calculating: ', end='')
    while t < 3:
        # rate(1/DT)
        VariableRobot.update_robot(r, t)
        # Monitor.update(r.mass_list, r.spring_list, p, l)
        # print(r.mass_list[0].position)
        results.append((copy.deepcopy(r.mass_list), copy.deepcopy(r.spring_list)))
        t = t + DT
        if i % 100 == 0:
            print('')
            i = 0
        else:
            print('.', end='')
        i = i + 1
    print('calculation done.')
    scene = canvas.get_selected()
    scene.waitfor('mouseup')
    for result in results:
        # rate((1 / DT) * 1.25)
        Monitor.update(result[0], result[1], p, l)
    print('animation done.')


if __name__ == '__main__':
    robot = VariableRobot.Robot()
    random_gene = VariableRobot.build_gene()
    gene = np.array([[[2, 4, 3],
                      [2, 1, 3],
                      [3, 2, 3]],

                     [[4, 4, 4],
                      [1, 2, 4],
                      [1, 2, 4]],

                     [[2, 4, 3],
                      [3, 2, 2],
                      [1, 2, 3]]])
    gene2 = np.array([[[4, 1, 3],
                       [4, 2, 3],
                       [3, 3, 4]],

                      [[1, 1, 1],
                       [1, 2, 3],
                       [4, 2, 3]],

                      [[1, 3, 3],
                       [3, 4, 4],
                       [4, 4, 3]]])
    robot.build_robot(random_gene)
    for mass in robot.mass_list:
        mass.position += vector(0, 0, 0.5)
    animate(robot)
