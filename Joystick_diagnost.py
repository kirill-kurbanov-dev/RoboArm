"""
Joystick control
"""
import os
import time
import socket
import pygame
import sys

import math3d as m3d
from math import pi
import multiprocessing as mp
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime

import urx
from urx_compat import patch_urx_math3d
patch_urx_math3d()
from config import load_config

from copy import deepcopy


def pose_to_list(pose):
    return [float(value) for value in pose]

auto = 0
f_lock = None
SP = []
p_lock = None
proc_logger = None
hb_queue = None
last_heartbeat = None
last_robot_activity = None
operation_timeout = 10.0

class Cmd(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self.axis0 = 0
        self.axis1 = 0
        self.axis2 = 0 
        self.axis3 = 0
        self.btn0 = 0
        self.btn1 = 0
        self.btn2 = 0
        self.btn3 = 0
        self.btn4 = 0
        self.btn5 = 0
        self.btn6 = 0
        self.btn7 = 0
        self.btn8 = 0
        self.btn9 = 0
        self.btn10 = 0
        self.btn11 = 0
        self.hat0 = [0,0] 


class Service(object):
    def __init__(self, linear_velocity=0.3, rotational_velocity=0.1, acceleration=0.1):
        global robot
        self.joystick = None
        self.robot = robot
        #max velocity and acceleration to be send to robot
        self.linear_velocity = linear_velocity
        self.rotational_velocity = rotational_velocity
        self.acceleration = acceleration
        #one button send the robot to a preprogram position defined by this variable in join space
        #self.init_pose = [-2.0782002408411593, -1.6628931459654561, 2.067930303382134, -1.9172217394630149, 1.5489023943220621, 0.6783171005488982]

        self.cmd = Cmd()


    def init_joystick(self):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(1)
        self.joystick.init()
        print('Initialized Joystick : %s' % self.joystick.get_name())

    def get_btn_state(self):
        self.cmd.reset()
        pygame.event.pump()  # Seems we need polling in pygame...

        # get joystick state
        for i in range(0, self.joystick.get_numaxes()):
            val = self.joystick.get_axis(i)
            if i in (2, 5) and val != 0:
                val += 1
            if abs(val) < 0.2:
                val = 0
            tmp = "self.cmd.axis" + str(i) + " = " + str(val)
            if val != 0:
                # print(tmp)
                exec(tmp)

        # get button state
        for i in range(0, self.joystick.get_numbuttons()):
            if self.joystick.get_button(i) != 0:
                tmp = "self.cmd.btn" + str(i) + " = 1"
                # print(tmp)
                exec(tmp)

    def loop(self, initiated_by = None):
        print("Starting diagnost loop")
        air = False
        stop = False
        path = []
        following_path = False
        global last_heartbeat, last_robot_activity, operation_timeout, hb_queue
        while True:
            current_time = time.time()
            if current_time - last_heartbeat >2.0:
                hb_queue.put((mp.current_process().name, "ALIVE", time.time()))
                last_heartbeat = current_time

            if not auto.value:
                continue
            if f_lock.value or p_lock.value:
                if not stop:
                    self.robot.speedl_tool([0]*6, 0.5, 2)
                    self.robot.stopl()
                    stop = True
                continue
            stop = False
            self.get_btn_state()
            #initalize speed array to 0
            speeds = [0, 0, 0, 0, 0, 0]

            #get linear speed from joystick
            speeds[1] = 1 * self.joystick.get_hat(0)[1] * self.linear_velocity
            speeds[0] = 1 * self.joystick.get_hat(0)[0] * self.linear_velocity
            if self.cmd.btn2 and not self.cmd.btn3:
                speeds[2] = 1 * -self.linear_velocity
            if self.cmd.btn3 and not self.cmd.btn2:
                speeds[2] = 1 * self.linear_velocity

            #get rotational speed from joystick
            speeds[3] = -1 * self.cmd.axis1 * self.rotational_velocity
            speeds[4] = -1 * self.cmd.axis0 * self.rotational_velocity
            speeds[5] = self.cmd.axis3 * self.rotational_velocity


            #for some reasons everything is inversed
            speeds = [-i for i in speeds]
            #Now sending to robot. tol by default and base csys if btn2 is on
            #if speeds != [0 for _ in speeds]:
                #print("Sending ", speeds)
            try:
                if self.cmd.btn0:
                    self.robot.speedl_tool(speeds, 0.1, 0.5)
                else:
                    self.robot.speedl(speeds, 0.1, 0.5)
            except(urx.RobotException, TimeoutError, ConnectionError) as e:
                error_msg = f"{type(e).__name__}:{str(e)[:100]}"
                proc_logger.error(f"Ошибка приработе с роботом: {error_msg}")
                hb_queue.put((mp.current_process().name, "ERROR", error_msg, time.time()))
                self.robot.close()
                break

            if self.cmd.btn10:
                path.append(pose_to_list(self.robot.getl()))
                print('point')
            if self.cmd.btn11:
                following_path = True
                for pose in path:
                    print("moving")
                    self.robot.movel(pose, acc=1, vel=0.3)
                    while self.robot.is_program_running():
                        self.get_btn_state()
                        if self.cmd.btn11:
                            following_path = False
                            self.robot.stop()
                path = []
            if self.cmd.btn8:
                SP[0] = deepcopy(path)

            #s.send((cmd).encode())
            #data = s.recv(1024)

            if initiated_by:
                if initiated_by.auto == False:
                    break

    def close(self):
        if self.joystick:
            self.joystick.quit()


def main(qu, a, shared_path, fl, pl):
    global robot, auto, SP, f_lock, p_lock, hb_queue, proc_logger, last_robot_activity, last_heartbeat, operation_timeout
    SP = shared_path
    auto = a
    f_lock = fl
    p_lock = pl

    hb_queue = qu

    proc_logger = logging.getLogger(f"RobotControl.{mp.current_process().name}")
    proc_logger.setLevel(logging.DEBUG)

    if not proc_logger.handlers:
        formatter = logging.Formatter(
            fmt='%(asctime)s.%(msecs)03d | %(levelname)-8s | %(processName)-15s | %(threadName)-15s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        file_handler = RotatingFileHandler(
            filename=f'logs/processes_status_log_{datetime.now().strftime("%Y-%m-%d")}.log',
            maxBytes=10 * 1024 * 1024,
            backupCount=5,
            encoding='utf-8'
        )

        file_handler.setFormatter(formatter)
        proc_logger.addHandler(file_handler)

    proc_logger.info(f"Запуск рабочего процесса | PID: {os.getpid()}")

    proc_logger.debug("Подключение к роботу...")
    robot = urx.Robot(load_config().diagnost_ip)
    r = robot
    proc_logger.info(f"Успешное подключение к роботу")

    hb_queue.put((mp.current_process().name, "READY", time.time()))
    proc_logger.debug("Отправлен статус READY в очередь сердцебиения")
    last_heartbeat = time.time()
    last_robot_activity = time.time()
    operation_timeout = 10.0


    service = Service(linear_velocity=0.01, rotational_velocity=0.19, acceleration=0.1)
    service.init_joystick()
    try:
        service.loop()
    finally:
        print('Джойстик диагноста отключён')
        service.close()

if __name__ == "__main__":
    robot = urx.Robot(load_config().diagnost_ip)
    r = robot

    service = Service(linear_velocity=0.01, rotational_velocity=0.19, acceleration=0.1)
    service.init_joystick()

    try:
        service.loop() 
    finally: 
        service.close()
