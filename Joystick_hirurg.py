"""
Joystick control
"""
"""
Импортирование библиотек
"""
import time
import socket
import pygame

import math3d as m3d
from math import pi, cos

import urx
from urx_compat import patch_urx_math3d
patch_urx_math3d()
from config import load_config
from threading import Thread
import Joystick_diagnost as diag
import multiprocessing as mp
from copy import deepcopy


def pose_to_list(pose):
    return [float(value) for value in pose]
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
import os

auto = None # Флаг режима управления роботами-манипуляторами True - режим синхронного управления, False - Режим асинхронного управления

f_lock = None
p_lock = None
SP = []

proc_logger = None
hb_queue = None
last_heartbeat = None
last_robot_activity = None
operation_timeout = 10.0

print(urx.__version__)  # Вывод текущей версии модуля urx


class Cmd(
    object):  # класс, содержащий состояния всех кнопок, отклонения по осям контроллера и миниджойстика контроллера
    def __init__(self):
        self.reset()  # сброс в ноль всех состояний

    def reset(self):
        # отклонения по осям
        self.axis0 = 0
        self.axis1 = 0
        self.axis2 = 0
        self.axis3 = 0
        # состояния кнопок
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
        # положение миниджойстика
        self.hat0 = [0, 0]


class Service(object):  # основной класс, осуществляющий управление роботом-манипулятором
    def __init__(self, linear_velocity=0.3, rotational_velocity=0.1, acceleration=0.1, robot = None):  # конструктор класса
        self.joystick = None  # переменная для хранения объекта контроллера
        self.robot = robot  # переменная для хранения объекта робота
        self.diagnost = urx.Robot(load_config().diagnost_ip)  # создание объекта диагноста и установление соединения с ним
        # max velocity and acceleration to be send to robot
        self.linear_velocity = linear_velocity  # максимальная линейная скорость
        self.rotational_velocity = rotational_velocity  # максимальная круговая скорость
        self.acceleration = acceleration  # ускорение
        # one button send the robot to a preprogram position defined by this variable in join space
        # self.init_pose = [-2.0782002408411593, -1.6628931459654561, 2.067930303382134, -1.9172217394630149, 1.5489023943220621, 0.6783171005488982]

        self.cmd = Cmd()  # объект состояний конетроллера

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

    def init_joystick(self):  # метод инициализации контроллера
        pygame.init()  # инициализация модуля pygame
        self.joystick = pygame.joystick.Joystick(0)  # создание объекта контроллера
        self.joystick.init()  # инициализация конроллера
        print('Initialized Joystick : %s' % self.joystick.get_name())  # вывод имени контроллера

    def loop(self):  # основной метод, в котором осуществляется управление
        print("Starting loop")
        global last_heartbeat
        air = False
        stop = False
        path = []
        while True:  # бесконечный цикл
            current_time = time.time()
            if current_time - last_heartbeat > 2.0:
                hb_queue.put((mp.current_process().name, "ALIVE", time.time()))
                last_heartbeat = current_time
            if p_lock.value:
                if not stop:
                    self.robot.speedl_tool([0] * 6, 0.5, 2)
                    self.robot.stopl()
                    stop = True
                continue
            stop = False
            self.cmd.reset()  # сброс состояний контроллера
            pygame.event.pump()  # поллинг ивентов
            hpos = self.robot.getl()  # текущая позиция хирурга
            dpos = self.diagnost.getl()  # текущая позиция диагноста

            self.get_btn_state()

            # initalize speed array to 0
            speeds = [0, 0, 0, 0, 0, 0]  # массив скоростей

            # get linear speed from joystick
            speeds[1] = 1 * self.joystick.get_hat(0)[1] * self.linear_velocity  # скорость по оси у
            speeds[0] = 1 * self.joystick.get_hat(0)[0] * self.linear_velocity  # скорость по оси х
            if self.cmd.btn2 and not self.cmd.btn3:  # движение по оси z
                speeds[2] = 1 * -self.linear_velocity  # скорость по оси z
            if self.cmd.btn3 and not self.cmd.btn2:  # движение против оси z
                speeds[2] = 1 * self.linear_velocity  # скорость по оси z

            # get rotational speed from joystick
            speeds[3] = -1 * self.cmd.axis1 * self.rotational_velocity  # круговая скорость в плоскости xy
            speeds[4] = -1 * self.cmd.axis0 * self.rotational_velocity  # круговая скорость в плоскости xz
            speeds[5] = self.cmd.axis3 * self.rotational_velocity  # круговая скорость в плоскости yz

            # for some reasons everything is inversed
            speeds = [-i for i in speeds]
            # Now sending to robot. tol by default and base csys if btn2 is on
            # if speeds != [0 for _ in speeds]:
            # print("Sending ", speeds)
            try:
                if self.cmd.btn0:  # если зажат курок
                    self.robot.speedl_tool(speeds, 0.1, 2)  # перемещение хирурга в системе координат конечного звена
                    T = self.robot.get_pose().array  # матрица перехода основание-конечное звено
                    # запись элементов матрицы в переменные
                    z1, z2 = -T[0][2], -T[1][2]
                    yx, yy = T[0][1], -T[1][1]
                    xx, xy = T[0][0], -T[1][0]
                    if not auto.value: # если включён режим синхронного перемещения
                        self.diagnost.speedl(((speeds[2] * z1 + speeds[0] * xx + speeds[1] * yx), # расчёт проекций скоростей на плоскость стола и перемещение диагноста с этими скоростями
                                                   speeds[2] * z2 + speeds[0] * xy + speeds[1] * yy, 0, 0, 0, 0), 0.1, 2)
                        print(speeds[2] * z1 + speeds[0] * xx + speeds[1] * yx, # вывод получившихся проекций
                              speeds[2] * z2 + speeds[0] * xy + speeds[1] * yy)
                        print(z1, xx, yx, xy)
                else:
                    self.robot.speedl(speeds, 0.1, 2) # если курок не зажат, то хирург перемещается в системе координат основания
            except(urx.RobotException, TimeoutError, ConnectionError) as e:
                error_msg = f"{type(e).__name__}:{str(e)[:100]}"
                proc_logger.error(f"Ошибка приработе с роботом: {error_msg}")
                hb_queue.put((mp.current_process().name, "ERROR", error_msg, time.time()))
                break
            # cmd = "speedl([{speeds}], 0.1, 5)\n".format(speeds=speeds)
            # s.send((cmd).encode())
            # data = s.recv(1024)

            if self.cmd.btn6 and (not self.cmd.btn7) and auto.value == 1: # включение синхронного режима
                auto.value = 0
                proc_logger.info(f"Система переведена в синхронный режим")
            if self.cmd.btn7 and (not self.cmd.btn6) and auto.value == 0: # включение асинхронного режима
                auto.value = 1
                #t = Thread(target=diag.main, args=[self]) # создание потока программы управления диагноста
                #t.start() # запуск потока
                proc_logger.info(f"Система переведена в асинхронный режим")
            if self.cmd.btn8:
                if not auto.value:
                    self.diagnost.movel((dpos[0], dpos[1], dpos[2], 0, 3.14, 0), 0.2, 0.2) # выравнивание хирурга

            if self.cmd.btn10:
                path.append(pose_to_list(self.robot.getl()))
                proc_logger.debug(f"Записана точка {path[-1]}")
            if self.cmd.btn11:
                following_path = True
                for pose in path:
                    proc_logger.debug(f"Перемещение в точку с координатами {pose}")
                    self.robot.movel(pose, acc=0.2, vel=0.2)
                    while self.robot.is_program_running():
                        self.get_btn_state()
                        if self.cmd.btn11:
                            following_path = False
                            self.robot.stop()

            if self.cmd.btn9:
                SP[0] = deepcopy(path)
                proc_logger.debug(f"Записана траектория {SP[0]}")
                path = []
                time.sleep(2)

    def close(self): # метод остановки программы
        if self.joystick:
            self.joystick.quit()
        self.robot.close()
        self.diagnost.close()


def main(qu, a, pl, shared_path):
    global auto, p_lock, SP, hb_queue, proc_logger, last_robot_activity, last_heartbeat, operation_timeout
    hb_queue = qu
    last_heartbeat = time.time()
    last_robot_activity = time.time()
    operation_timeout = 10.0

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
    robot = urx.Robot(load_config().surgeon_ip)  # установление соединения с хирургом
    proc_logger.info(f"Успешное подключение к роботу")
    auto = a
    p_lock = pl
    SP = shared_path
    print(auto, "\n", a)

    service = Service(linear_velocity=0.015, rotational_velocity=0.19, acceleration=0.1,
                      robot=robot)  # создание объекта класса управления роботами
    service.init_joystick()  # инициализация контроллера
    service.loop()
    service.close()

if __name__ == "__main__": # точка входа в программу
    main(0)
