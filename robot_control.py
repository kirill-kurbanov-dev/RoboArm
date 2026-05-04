import logging
from logging.handlers import RotatingFileHandler
import multiprocessing as mp
import os
import socket
import time
from datetime import datetime

import pygame

from config import load_config


def as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class DirectURRobot:
    def __init__(self, host, port, logger):
        self.host = host
        self.port = int(port)
        self.logger = logger
        self.sock = None
        self.last_program = None

    def connect(self):
        self.close()
        self.logger.info(f"Подключение URScript {self.host}:{self.port}")
        self.sock = socket.create_connection((self.host, self.port), timeout=3)
        self.sock.settimeout(1)
        self.logger.info(f"URScript подключён {self.host}:{self.port}")

    def send(self, program):
        if self.sock is None:
            self.connect()
        try:
            self.sock.sendall(program.encode("utf-8"))
        except OSError:
            self.logger.warning(f"Переоткрытие URScript соединения {self.host}:{self.port}")
            self.connect()
            self.sock.sendall(program.encode("utf-8"))
        self.last_program = program

    def speedl(self, speeds, acceleration, duration):
        values = ", ".join(f"{float(value):.6f}" for value in speeds)
        self.send(f"speedl([{values}], {float(acceleration):.6f}, {float(duration):.6f})\n")

    def stop(self):
        try:
            self.speedl([0.0] * 6, 0.2, 0.1)
            self.send("stopl(0.200000)\n")
        except Exception:
            self.logger.exception(f"Не удалось остановить {self.host}")

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


class JoystickRobotController:
    def __init__(self, host, is_master, logger, auto, lock, shared_path, heartbeat, slave_host=None):
        self.config = load_config()
        self.host = host
        self.is_master = is_master
        self.logger = logger
        self.auto = auto
        self.lock = lock
        self.shared_path = shared_path
        self.heartbeat = heartbeat
        self.slave_host = slave_host
        self.control_port = int(getattr(self.config, "control_port", 30002))
        self.deadzone = as_float(self.config.deadzone, 0.2)
        self.linear_velocity = as_float(
            self.config.linear_velocity if is_master else self.config.diagnost_linear_velocity,
            0.015 if is_master else 0.01,
        )
        self.rotational_velocity = as_float(self.config.rotational_velocity, 0.19)
        self.acceleration = as_float(self.config.acceleration, 0.1)
        self.duration = 0.12
        self.last_heartbeat = time.time()
        self.joystick = None
        self.axis_neutral = []
        self.robot = DirectURRobot(self.host, self.control_port, self.logger)
        self.slave = None
        self.was_moving = False

    def init_joystick(self):
        pygame.init()
        pygame.joystick.init()
        joystick_id = self.config.joystick_master_id if self.is_master else self.config.joystick_slave_id
        joystick_id = int(joystick_id)
        count = pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError("Джойстик не найден")
        if joystick_id >= count:
            self.logger.warning(f"Джойстик id={joystick_id} не найден, используется id=0 из {count}")
            joystick_id = 0
        self.joystick = pygame.joystick.Joystick(joystick_id)
        self.joystick.init()
        pygame.event.pump()
        time.sleep(0.1)
        pygame.event.pump()
        self.axis_neutral = [
            as_float(self.joystick.get_axis(axis_id), 0.0)
            for axis_id in range(self.joystick.get_numaxes())
        ]
        self.logger.info(
            f"Инициализирован джойстик id={joystick_id}, name={self.joystick.get_name()}, "
            f"axes={self.joystick.get_numaxes()}, buttons={self.joystick.get_numbuttons()}, hats={self.joystick.get_numhats()}, "
            f"axis_neutral={[round(value, 3) for value in self.axis_neutral]}"
        )

    def ensure_slave(self):
        if not self.is_master or not self.slave_host:
            return None
        if self.slave is None:
            self.slave = DirectURRobot(self.slave_host, self.control_port, self.logger)
            self.slave.connect()
        return self.slave

    def button_pressed(self, button_id):
        return self.joystick.get_numbuttons() > button_id and bool(self.joystick.get_button(button_id))

    def axis(self, axis_id):
        if self.joystick.get_numaxes() <= axis_id:
            return 0.0
        neutral = self.axis_neutral[axis_id] if axis_id < len(self.axis_neutral) else 0.0
        value = as_float(self.joystick.get_axis(axis_id), 0.0) - neutral
        value = max(-1.0, min(1.0, value))
        return value if abs(value) >= self.deadzone else 0.0

    def hat(self):
        if self.joystick.get_numhats() == 0:
            return 0.0, 0.0
        x, y = self.joystick.get_hat(0)
        return as_float(x), as_float(y)

    def read_speeds(self):
        pygame.event.pump()
        hat_x, hat_y = self.hat()
        speeds = [0.0] * 6

        speeds[0] = hat_x * self.linear_velocity
        speeds[1] = hat_y * self.linear_velocity

        if self.button_pressed(2) and not self.button_pressed(3):
            speeds[2] = -self.linear_velocity
        elif self.button_pressed(3) and not self.button_pressed(2):
            speeds[2] = self.linear_velocity

        speeds[3] = -self.axis(1) * self.rotational_velocity
        speeds[4] = -self.axis(0) * self.rotational_velocity
        speeds[5] = self.axis(3) * self.rotational_velocity

        return [float(value) for value in speeds]

    @staticmethod
    def has_motion(speeds):
        return any(abs(value) > 1e-6 for value in speeds)

    def update_heartbeat(self, state="ALIVE"):
        now = time.time()
        if state == "READY" or now - self.last_heartbeat >= 2.0:
            self.heartbeat.put((mp.current_process().name, state, time.time()))
            self.last_heartbeat = now

    def start(self):
        self.init_joystick()
        self.robot.connect()
        self.update_heartbeat("READY")
        self.logger.info(
            f"Прямое управление запущено: host={self.host}, port={self.control_port}, "
            f"linear={self.linear_velocity}, rotational={self.rotational_velocity}, acceleration={self.acceleration}"
        )
        self.loop()

    def loop(self):
        while True:
            self.update_heartbeat()

            if self.lock.value:
                if self.was_moving:
                    self.robot.stop()
                    if self.slave:
                        self.slave.stop()
                    self.was_moving = False
                time.sleep(0.05)
                continue

            speeds = self.read_speeds()

            if self.has_motion(speeds):
                self.robot.speedl(speeds, self.acceleration, self.duration)
                if self.is_master and self.auto.value == 0 and self.button_pressed(0):
                    slave = self.ensure_slave()
                    if slave:
                        slave.speedl(speeds, self.acceleration, self.duration)
                self.was_moving = True
            else:
                if self.was_moving:
                    self.robot.stop()
                    if self.slave:
                        self.slave.stop()
                    self.was_moving = False
                time.sleep(0.02)

            if self.button_pressed(6) and not self.button_pressed(7) and self.auto.value == 1 and self.is_master:
                self.auto.value = 0
                self.logger.info("Система переведена в синхронный режим")
                time.sleep(0.3)
            elif self.button_pressed(7) and not self.button_pressed(6) and self.auto.value == 0 and self.is_master:
                self.auto.value = 1
                self.logger.info("Система переведена в асинхронный режим")
                time.sleep(0.3)

    def close(self):
        if self.was_moving:
            self.robot.stop()
            if self.slave:
                self.slave.stop()
        if self.joystick:
            self.joystick.quit()
        self.robot.close()
        if self.slave:
            self.slave.close()
        pygame.quit()


def setup_process_logger():
    logger = logging.getLogger(f"RobotControl.{mp.current_process().name}")
    logger.setLevel(logging.DEBUG)
    if logger.handlers:
        return logger

    os.makedirs("logs", exist_ok=True)
    formatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d | %(levelname)-8s | %(processName)-15s | %(threadName)-15s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    file_handler = RotatingFileHandler(
        filename=f'logs/processes_status_log_{datetime.now().strftime("%Y-%m-%d")}.log',
        maxBytes=10 * 1024 * 1024,
        backupCount=5,
        encoding="utf-8",
    )
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    return logger


def main(IP, is_master, auto, lock, shared_path, heartbeat, slave_IP=None):
    logger = setup_process_logger()
    logger.info(f"Запуск прямого управления | PID: {os.getpid()}")
    controller = JoystickRobotController(IP, is_master, logger, auto, lock, shared_path, heartbeat, slave_IP)
    try:
        controller.start()
    except Exception as exc:
        message = f"{type(exc).__name__}: {str(exc)[:200]}"
        logger.exception(f"Процесс управления остановлен: {message}")
        heartbeat.put((mp.current_process().name, "ERROR", message, time.time()))
    finally:
        controller.close()
