import time
import pygame
import urx
from urx_compat import patch_urx_math3d
patch_urx_math3d()
import multiprocessing as mp
from copy import deepcopy
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
import os

from config import load_config


class Robot:

    def __init__(self, IP, is_master, logger, auto, lock, shared_path, heartbeat, slave_IP=None):
        self.config = load_config()
        self.IP = IP
        self.is_master = is_master
        self.logger = logger
        self.auto = auto
        self.lock = lock
        self.shared_path = shared_path
        self.heartbeat = heartbeat
        self.slave_IP = slave_IP
        self.path = []
        self.last_heartbeat = time.time()
        self.robot = None
        self.slave = None
        self.joystick = None
        self.deadzone = self.config.deadzone

    @staticmethod
    def pose_to_list(pose):
        return [float(value) for value in pose]

    def init_robot(self):
        self.logger.debug(f"Подключение к роботу IP: {self.IP}...")
        self.robot = urx.Robot(self.IP)
        self.logger.info(f"Успешное подключение к роботу IP: {self.IP}")
        if self.is_master:
            if self.slave_IP:
                self.logger.debug(f"Подключение к роботу slave IP: {self.slave_IP}...")
                self.slave = urx.Robot(self.slave_IP)
                self.logger.info(f"Успешное подключение к роботу slave IP: {self.slave_IP}")
            else:
                self.logger.error(f"Отсутствует slave IP")
                self.heartbeat.put((mp.current_process().name, "ERROR", "Отсутствует slave IP", time.time()))
                exit(1)

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()

        if self.is_master:
            joystick_id = self.config.joystick_master_id
        else:
            joystick_id = self.config.joystick_slave_id

        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise RuntimeError("Джойстик не найден")
        if joystick_id >= joystick_count:
            self.logger.warning(
                f"Джойстик id={joystick_id} не найден, используется id=0 из {joystick_count} доступных"
            )
            joystick_id = 0

        self.joystick = pygame.joystick.Joystick(joystick_id)
        self.joystick.init()
        self.logger.info(f"Инициализирован джойстик id={joystick_id}, name: {self.joystick.get_name()}")

    def start_robot(self):
        self.init_joystick()
        self.init_robot()
        self.set_robot_settings()
        self.main_loop()

    def set_robot_settings(self, linear_velocity=0.3, rotational_velocity=0.1, acceleration=0.1):
        if self.is_master:
            self.linear_velocity = self.config.linear_velocity
        else:
            self.linear_velocity = self.config.diagnost_linear_velocity
        self.rotational_velocity = self.config.rotational_velocity
        self.acceleration = self.config.acceleration

    def get_speeds(self):
        pygame.event.pump()
        speeds = [0, 0, 0, 0, 0, 0]  # массив скоростей

        hat = self.joystick.get_hat(0) if self.joystick.get_numhats() > 0 else (0, 0)
        speeds[1] = 1 * hat[1] * self.linear_velocity  # скорость по оси у
        speeds[0] = 1 * hat[0] * self.linear_velocity  # скорость по оси х
        if self.button_pressed(2) and not self.button_pressed(3):  # движение по оси z
            speeds[2] = 1 * -self.linear_velocity  # скорость по оси z
        if self.button_pressed(3) and not self.button_pressed(2):  # движение против оси z
            speeds[2] = 1 * self.linear_velocity  # скорость по оси z

        axes_count = self.joystick.get_numaxes()
        axis0 = self.joystick.get_axis(0) if axes_count > 0 else 0
        axis1 = self.joystick.get_axis(1) if axes_count > 1 else 0
        axis3 = self.joystick.get_axis(3) if axes_count > 3 else 0

        speeds[3] = (axis1 if abs(
            axis1) > self.deadzone else 0) * self.rotational_velocity  # круговая скорость в плоскости xy
        speeds[4] = (axis0 if abs(
            axis0) > self.deadzone else 0) * self.rotational_velocity  # круговая скорость в плоскости xz
        speeds[5] = (axis3 if abs(
            axis3) > self.deadzone else 0) * self.rotational_velocity  # круговая скорость в плоскости yz

        return speeds

    def button_pressed(self, button_id):
        return self.joystick.get_numbuttons() > button_id and self.joystick.get_button(button_id)

    def update_heartbeat(self):
        current_time = time.time()
        if current_time - self.last_heartbeat > 2.0:
            self.heartbeat.put((mp.current_process().name, "ALIVE", time.time()))
            self.last_heartbeat = current_time

    def get_slave_speeds(self):
        if not self.is_master:
            return
        T = self.robot.get_pose().array  # матрица перехода основание-конечное звено
        # запись элементов матрицы в переменные
        z1, z2 = -T[0][2], -T[1][2]
        yx, yy = T[0][1], -T[1][1]
        xx, xy = T[0][0], -T[1][0]

        speeds = [self.speeds[2] * z1 + self.speeds[0] * xx + self.speeds[1] * yx,
                  self.speeds[2] * z2 + self.speeds[0] * xy + self.speeds[1] * yy, 0, 0, 0, 0]
        return speeds

    def main_loop(self):
        self.running = True
        stop = False
        pygame.event.pump()

        while self.running:

            self.update_heartbeat()

            if self.lock.value:
                if not stop:
                    self.robot.speedl_tool([0] * 6, 0.5, 2)
                    self.robot.stopl()
                    stop = True
                continue
            else:
                stop = False

            self.pos = self.pose_to_list(self.robot.getl())
            if self.is_master:
                self.slave_pos = self.pose_to_list(self.slave.getl())

            self.speeds = self.get_speeds()

            try:
                if self.button_pressed(0):  # если зажат курок
                    self.robot.speedl_tool(self.speeds, 0.1,
                                           2)  # перемещение робота в системе координат конечного звена
                    if self.auto.value == 0:  # если включён режим синхронного перемещения
                        if self.is_master:
                            self.slave_speeds = self.get_slave_speeds()
                            self.slave.speedl(self.slave_speeds, 0.1, 2)
                else:
                    self.robot.speedl(self.speeds, 0.1,
                                      2)  # если курок не зажат, то робот перемещается в системе координат основания

            except(urx.RobotException, TimeoutError, ConnectionError) as e:
                error_msg = f"{type(e).__name__}:{str(e)[:100]}"
                self.logger.error(f"Ошибка приработе с роботом IP: {self.IP}, сообщение: {error_msg}")
                self.heartbeat.put((mp.current_process().name, "ERROR", error_msg, time.time()))
                break

            if self.button_pressed(6) and (not self.button_pressed(
                    7)) and self.auto.value == 1 and self.is_master:  # включение синхронного режима
                self.auto.value = 0
                self.logger.info(f"Система переведена в синхронный режим")
            if self.button_pressed(7) and (not self.button_pressed(
                    6)) and self.auto.value == 0 and self.is_master:  # включение асинхронного режима
                self.auto.value = 1
                self.logger.info(f"Система переведена в асинхронный режим")
            if self.button_pressed(8):
                if self.auto.value == 0:
                    if self.is_master:
                        self.slave.movel((self.slave_pos[0], self.slave_pos[1], self.slave_pos[2], 0, 3.14, 0), 0.2,
                                         0.2)  # выравнивание хирурга

            if self.button_pressed(10):
                self.path.append(self.pose_to_list(self.robot.getl()))
                self.logger.debug(f"Записана точка {self.path[-1]}")
            if self.button_pressed(11):
                following_path = True
                for pose in self.path:
                    self.logger.debug(f"Перемещение в точку с координатами {pose}")
                    self.robot.movel(pose, acc=0.2, vel=0.2)
                    while self.robot.is_program_running():
                        pygame.event.pump()
                        if self.button_pressed(11):
                            following_path = False
                            self.robot.stop()

            if self.button_pressed(9):
                self.shared_path[0] = deepcopy(self.path)
                self.logger.debug(f"Записана траектория {self.shared_path[0]}")
                self.path = []
                time.sleep(2)

    def close(self):
        if self.joystick:
            self.joystick.quit()
        if self.robot:
            self.robot.close()
        if self.is_master and self.slave:
            self.slave.close()


def main(IP, is_master, auto, lock, shared_path, heartbeat, slave_IP=None):
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

    robot = Robot(IP, is_master, proc_logger, auto, lock, shared_path, heartbeat, slave_IP)
    try:
        robot.start_robot()
    except Exception as exc:
        error_msg = f"{type(exc).__name__}: {str(exc)[:200]}"
        proc_logger.exception(f"Процесс управления остановлен: {error_msg}")
        heartbeat.put((mp.current_process().name, "ERROR", error_msg, time.time()))
    finally:
        robot.close()
