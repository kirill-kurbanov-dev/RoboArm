import urx
from urx_compat import patch_urx_math3d
patch_urx_math3d()
import time
import socket
from math import pi
from config import load_config

import multiprocessing as mp

d = mp.Value("i", 32767)
h = mp.Value("i", 32767)

def rob_init():
    cfg = load_config()

    hirurg = urx.Robot(cfg.surgeon_ip)
    diagnost = urx.Robot(cfg.diagnost_ip)

    time.sleep(5)

    d_pose = diagnost.getl()
    diagnost.movel((d_pose[0], d_pose[1], d_pose[2], 0, 3.14, 0), acc=0.2, vel=0.2)

    h_pose = hirurg.getl()
    hirurg.movel((h_pose[0], h_pose[1], h_pose[2], 3.14 / 2, 0, 0), acc=0.2, vel=0.2)

    return diagnost, hirurg

def set_hirurg_angle(hirurg, angle):
    h_pose = hirurg.getl()
    hirurg.movel((h_pose[0], h_pose[1], h_pose[2], 3.14 / 2, 0, 0), acc=0.2, vel=0.2)
    while hirurg.is_program_running():
        pass

    if angle != 0:
        angle *= (pi / 180)
        o = hirurg.get_orientation()
        o.rotate_xt(angle)
        hirurg.set_orientation(o, 0.1, 0.1)
        while hirurg.is_program_running():
            pass
    else:
        return -1

def ones_complement_to_signed(value, bit_length = 16):
    if value & (1 << (bit_length-1)):
        value = ~value & ((1<< bit_length)-1)
        value = -(value +1)
    return value

def get_dist(d, h):
    cfg = load_config()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # конфигурация соединения
    sock.connect((cfg.laser_host, cfg.laser_port))
    while True:
        data = sock.recv(16)  # получение данных
        ts = data.decode()
        dd = ts.split('end')
        hh = dd[1].split('end')
        # print(dd)
        d.value = ones_complement_to_signed(int(dd[0]))
        h.value = ones_complement_to_signed(int(hh[0]))
        # print(hh)
    # return ones_complement_to_signed(int(d[0])), ones_complement_to_signed(int(h[0]))

def get_av_dist(cnt):
    global d, h
    sum_d = 0
    sum_h = 0

    for k in range(cnt):
        sum_d += d.value
        sum_h += h.value
        # time.sleep(1)

    return int(sum_d/cnt) , int(sum_h/cnt)

def find_start(diagnost, hirurg):
    diagnost.translate_tool((0, 0, -0.02), 0.005, 0.005)
    while diagnost.is_program_running():
        pass

    while get_av_dist(10)[0] == 32767:
        # print(d.value, get_av_dist(10)[0])
        diagnost.translate_tool((0, 0, 0.001), 0.001, 0.001)
        while diagnost.is_program_running():
            pass

    # hirurg.translate_tool((0, 0, -0.02), 0.005, 0.005)
    # while hirurg.is_program_running():
    #     pass
    # print(get_av_dist(100)[1])
    # while get_av_dist(10)[1] == 32767:
    #     print(get_av_dist(10)[1])
    #     hirurg.translate_tool((0, 0, 0.001), 0.001, 0.001)
    #     while hirurg.is_program_running():
    #         pass








def experiment_uno(diagnost, hirurg):

    for x in range(10):
        step = 0.001*(x+1)
        for k in range(10):
            with open(f'Э1Д/шаг {x+1} мм {k}.txt', "w") as file:
                start_pose = diagnost.getl()[2]
                for i in range(10):
                    diagnost.translate_tool((0, 0, step*(-1)**k), 0.001, 0.001)
                    while diagnost.is_program_running():
                        pass
                    file.write(f'{i + 1} {diagnost.getl()[2] - start_pose} {get_av_dist(10)[0]}\n')

    # for x in range(10):
    #     step = 0.001*(x+1)
    #     for k in range(10):
    #         with open(f'Э1Х/шаг {x+1} мм {k}.txt', "w") as file:
    #             start_pose_x = hirurg.getl()[1]
    #             for i in range(10):
    #                 hirurg.translate_tool((0, 0, step*(-1)**k), 0.001, 0.001)
    #                 while hirurg.is_program_running():
    #                     pass
    #                 file.write(f'{i + 1} {hirurg.getl()[1] - start_pose_x} {get_av_dist(10)[1]}\n')

def experiment_duo(hirurg, diagnost):
    for x in range(10):
        step = 0.001*(x+1)
        for j in range(5):
            angle = 5*(j+1)
            set_hirurg_angle(hirurg, angle)
            find_start(diagnost, hirurg)
            for k in range(10):
                with open(f'Э2Х/шаг {x+1} мм, угол {angle} {k}.txt', "w") as file:
                    start_pose_x = hirurg.getl()[1]
                    start_pose_z = hirurg.getl()[2]
                    for i in range(10):
                        hirurg.translate_tool((0, 0, step*(-1)**k), 0.001, 0.001)
                        while hirurg.is_program_running():
                            pass
                        file.write(f'{i + 1} {hirurg.getl()[1] - start_pose_x} {hirurg.getl()[2]-start_pose_z} {get_av_dist(10)[1]}\n')

def experiment_tres(diagnost, hirurg):

    for x in range(10):
        speed = 0.001*(x+1)
        for k in range(6):
            with open(f'Э3Д/скорость {x + 1} мм {k}.txt', "w") as file:
                start_pose = diagnost.getl()[2]
                start_time = time.time()
                diagnost.translate_tool((0,0,0.1*(-1)**k), speed, speed)
                while hirurg.is_program_running():
                    pass
                end_time = time.time()
                end_pose = diagnost.getl()[2]
                elapsed_time = end_time - start_time
                delta = end_pose - start_pose
                file.write(f'{elapsed_time} {delta}\n')

    for x in range(10):
        speed = 0.001*(x+1)
        for k in range(6):
            with open(f'Э3Х/скорость {x + 1} мм {k}.txt', "w") as file:
                start_pose = hirurg.getl()[2]
                start_time = time.time()
                hirurg.translate_tool((0,0,0.1*(-1)**k), speed, speed)
                while hirurg.is_program_running():
                    pass
                end_time = time.time()
                end_pose = hirurg.getl()[2]
                elapsed_time = end_time - start_time
                delta = end_pose - start_pose
                file.write(f'{elapsed_time} {delta}\n')










def main():
    global d, h
    D, H = rob_init()

    data_acquisitor = mp.Process(target=get_dist, args = (d,h))
    data_acquisitor.start()

    find_start(D, H)
    experiment_uno(D, H)
    # experiment_duo(H, D)
    # experiment_tres(D, H)


    D.close()
    H.close()




if __name__ == "__main__":


    main()
