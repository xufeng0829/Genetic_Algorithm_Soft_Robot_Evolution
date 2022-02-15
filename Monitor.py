"""
This is a program to monitor and display plots, 3-d animations and movies for robots dynamics.
"""

from vpython import *


def figure():
    canvas(title='fastest robot', width=1280, height=720, center=vector(0.05, 0.05, 0.05), background=color.black)
    box(pos=vector(0, 0, 0), size=vector(5, 5, 0.001), texture='https://i.imgur.com/sPvNjd5.jpeg')


def display(mass_list, spring_list):
    point_list = []
    line_list = []
    for p in mass_list:
        point_list.append(sphere(pos=p.position, radius=0.02, color=color.blue))
    for s in spring_list:
        line_list.append(cylinder(pos=mass_list[s.m1].position, axis=mass_list[s.m2].position-mass_list[s.m1].position,
                                  radius=0.005, color=color.red))
    return point_list, line_list


def update(mass_list, spring_list, point_list, line_list):
    cnt = 0
    for p in mass_list:
        point_list[cnt].pos = p.position
        cnt = cnt + 1
    cnt = 0
    for s in spring_list:
        line_list[cnt].pos = mass_list[s.m1].position
        line_list[cnt].axis = mass_list[s.m2].position-mass_list[s.m1].position
        cnt = cnt + 1


def writing_energy(t, energy: tuple, f):
    f.writerow([t, energy[0], energy[1], energy[2], energy[3]])
