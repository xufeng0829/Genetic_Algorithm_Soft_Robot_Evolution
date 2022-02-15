if __name__ == '__main__':
    g = build_gene()
    print(g)
    r = Robot()
    start = time.time()
    r.build_robot(g)
    com_start = COM(r)
    Monitor.figure()
    p1, l1 = Monitor.display(r.mass_list, r.spring_list)
    t = 0
    result_mass=[]
    result_spring = []
    scene = canvas.get_selected()
    scene.waitfor('mouseup')
    print('calculation start')
    while t < 3:
        # rate(1/DT)
        update_robot(r, t)
        # Monitor.update(r.mass_list, r.spring_list, p1, l1)
        t = t + DT
        t = round(t, 3)
        print(t)
    print("calculation done.")
    print('distance:')
    print(COM(r).x - com_start.x)
    print('time:')
    print(time.time() - start)
