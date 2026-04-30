import math
import time
import urx
from urx_compat import patch_urx_math3d
patch_urx_math3d()
from datetime import datetime
from config import load_config

def force_calibration(rob, vel, acc, cal_time):
    Fd = []
    Fu = []
    Fs = []
    start_pose = rob.getl()
    rob.speedl_tool([vel,0,0,0,0,0], acc, cal_time)
    start_time = time.time()
    while (time.time() - start_time) < cal_time:
        Fs.append(rob.get_tcp_force()[2])
    rob.speedl_tool([0] * 6, 0.5, 2)
    rob.stopl()
    Fs = sum(Fs)/len(Fs)

    rob.speedl_tool([0, 0, vel, 0, 0, 0], acc, cal_time)
    start_time = time.time()
    while (time.time() - start_time) < cal_time:
        Fd.append(rob.get_tcp_force()[2])
    rob.speedl_tool([0] * 6, 0.5, 2)
    rob.stopl()
    Fd = sum(Fd) / len(Fd)

    rob.speedl_tool([0, 0, -vel, 0, 0, 0], acc, cal_time)
    start_time = time.time()
    while (time.time() - start_time) < cal_time:
        Fu.append(rob.get_tcp_force()[2])
    rob.speedl_tool([0] * 6, 0.5, 2)
    rob.stopl()
    Fu = sum(Fu) / len(Fu)
    rob.movel(start_pose, acc = 0.05, vel = 0.05)
    return Fs, Fd, Fu

def FC_velocity(Fm, f_raw, angle_rad, f_target = 1.0, kf = 0.01, m_tool = 0.2, v_max = 0.05):

    # gravity_force = m_tool*9.81*math.cos(angle_rad)
    gravity_force = Fm
    f_contact = f_raw - gravity_force
    f_error = f_target - f_contact
    v_z = kf * f_error

    if v_z>v_max:
        v_z = v_max
    elif v_z < -v_max:
        v_z = -v_max
    return v_z





if __name__ == "__main__":
    rob = urx.Robot(load_config().diagnost_ip,use_rt=True)
    speeds = [0]*6
    vel = 0.005
    acc = 0.005
    f_target = 2
    Kf = 0.08
    move_time = 30
    Fm = force_calibration(rob, vel, acc, 3)

    start_pose = rob.getl()

    rob.speedl_tool([0,0,0.01,0,0,0], acc, 30)
    while (rob.get_tcp_force()[2] - Fm[0]) < f_target:
        pass
    rob.speedl_tool([0] * 6, 0.5, 2)
    rob.stopl()
    time.sleep(3)
    file_name = f'K_{Kf}_F_{f_target}.txt'
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_name = f'{timestamp}_{file_name}'
    with open(file_name, "w") as file:
        start_time = time.time()
        while time.time() - start_time < move_time:
            F = rob.get_tcp_force()
            force = F[2]
            v_z = FC_velocity(Fm[0], force, 0, f_target, Kf)
            rob.speedl_tool([vel, 0,v_z, 0, 0, 0], acc, 1)
            file.write(f'{force - Fm[0]} {v_z}\n')

    rob.speedl_tool([0] * 6, 0.5, 2)
    rob.stopl()
    rob.movel(start_pose, acc = 0.05, vel = 0.05)

    rob.close()


