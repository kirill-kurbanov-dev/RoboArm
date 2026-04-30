import tkinter as tk
from tkinter import ttk
import time
import urx
import math3d as m3d
from urx_compat import patch_urx_math3d
patch_urx_math3d()
import subprocess
import pygame
import socket
from config import load_config

def route_follow(nsteps, step_val, pause_time, vel):
    rob = urx.Robot(load_config().diagnost_ip, use_rt=True)
    force = [rob.get_tcp_force()[2]]
    t = [0]
    coordinates = [0]
    start = time.time()
    for x in range(nsteps):
        rob.translate((0, step_val , 0), 0.1, vel)
        while rob.is_program_running():
            pass
        t.append(time.time()-start)
        time.sleep(pause_time)
        coordinates.append(rob.getl()[0])
        force.append(rob.get_tcp_force()[2])
    for x in range(nsteps):
        rob.translate((0, -step_val , 0), 0.1, vel)
        while rob.is_program_running():
            pass
        t.append(time.time()-start)
        time.sleep(pause_time)
        coordinates.append(rob.getl()[1])
        force.append(rob.get_tcp_force()[2])
    with open('data.txt', 'w') as file:
        file.write("Step | Time,s | Coordinate | Force, N\n")
        for x in range(len(t)):
            file.write(f'{x} {t[x]} {coordinates[x]} {force[x]}\n')


class ScriptRunnerApp:
    def __init__(self, root):
        self.root = root
        self.root.title('Основное меню')

        tabControl = ttk.Notebook(root)
        control_tab = ttk.Frame(tabControl)
        route_tab = ttk.Frame(tabControl)

        tabControl.add(control_tab, text='Управление манипулятором')
        tabControl.add(route_tab, text='Путь')
        tabControl.pack(expand=1, fill="both")

        # self.label = tk.Label(root, text= "Запуск системы управления")

        self.system_launch_btn = tk.Button(control_tab, text='Запуск системы', command=self.system_launch, state=tk.NORMAL)
        self.system_launch_btn.grid(column=0, row=0, padx=5, pady=15)

        self.system_stop_btn = tk.Button(control_tab, text='Остановка системы', command=self.system_stop, state=tk.DISABLED)
        self.system_stop_btn.grid(column=1, row=0, padx=5, pady=15)

        self.control_launch_btn = tk.Button(control_tab, text='Запуск управления', command=self.control_launch,
                                            state=tk.DISABLED)
        self.control_launch_btn.grid(column=0, row=1, padx=10, pady=10)

        self.control_stop_btn = tk.Button(control_tab, text='Остановка управления', command=self.control_stop,
                                          state=tk.DISABLED)
        self.control_stop_btn.grid(column=0, row=2, padx=10, pady=10)

        self.align_btn = tk.Button(control_tab, text='Выравнивание робота', command=self.align, state=tk.DISABLED)
        self.align_btn.grid(column=1, row=1, padx=10, pady=10)

        self.unlock_btn = tk.Button(control_tab, text='Разблокировать роботов', command=self.unlock)
        self.unlock_btn.grid(column=0, row=3, columnspan=3, padx=10, pady=10)

        self.num_steps_label = tk.Label(route_tab, text = "Количество шагов").grid(column = 0 , row = 0)
        self.step_val_label = tk.Label(route_tab, text="Величина шага").grid(column=0, row=1, padx=10, pady=10)
        self.pause_time_label = tk.Label(route_tab, text="Время остановки").grid(column=0, row=2, padx=10, pady=10)
        self.velocity_label = tk.Label(route_tab, text="Скорость").grid(column=0, row=3, padx=10, pady=10)

        self.num_steps_entry = tk.Entry(route_tab)
        self.num_steps_entry.grid(column=1, row =0, padx = 10, pady =10)
        self.step_val_entry = tk.Entry(route_tab)
        self.step_val_entry.grid(column=1, row=1, padx=10, pady=10)
        self.pause_time_entry = tk.Entry(route_tab)
        self.pause_time_entry.grid(column=1, row=2, padx=10, pady=10)
        self.velocity_entry = tk.Entry(route_tab)
        self.velocity_entry.grid(column=1, row=3, padx=10, pady=10)

        self.start_route_btn = tk.Button(route_tab, text = "Начать маршрут", command=self.start_route).grid(column= 0, row = 4)

        self.control_process = None

    def system_launch(self):
        print('System launched')
        self.system_launch_btn.configure(state=tk.DISABLED)
        self.system_stop_btn.configure(state=tk.NORMAL)
        self.control_launch_btn.configure(state=tk.NORMAL)
        self.align_btn.configure(state=tk.NORMAL)
        self.power_on_d_process = subprocess.Popen(['python', 'Power_On_D.py'])

    def system_stop(self):
        print('system_stopped')
        self.system_launch_btn.configure(state=tk.NORMAL)
        self.system_stop_btn.configure(state=tk.DISABLED)
        self.control_stop_btn.configure(state=tk.DISABLED)
        self.control_launch_btn.configure(state=tk.DISABLED)
        self.align_btn.configure(state=tk.DISABLED)
        self.power_off_d_process = subprocess.Popen(['python', 'Power_Off_D.py'])

    def control_launch(self):
        self.control_stop_btn.configure(state=tk.NORMAL)
        self.control_launch_btn.configure(state=tk.DISABLED)
        self.control_process = subprocess.Popen(['python', 'Joystick_diagnost.py'])
        time.sleep(5)
        if self.control_process.poll() is None:
            print('control launched')
        else:
            print('Error while initiating control')
            self.control_stop()

    def control_stop(self):
        print('control stopped')
        self.control_stop_btn.configure(state=tk.DISABLED)
        self.control_launch_btn.configure(state=tk.NORMAL)
        if not (self.control_process.poll() is None):
            self.control_process.kill()

    def align(self):
        print('aligned')
        self.align_d_process = subprocess.Popen(['python', 'Align_D.py'])

    def unlock(self):
        print('unlocked')
        self.unlock_h_process = subprocess.Popen(['python', 'ESTOP_RESET_D.py'])

    def start_route(self):
        if not (self.control_process is None):
            self.control_process.kill()
        nsteps = int(self.num_steps_entry.get())
        stepval = float(self.step_val_entry.get())
        pause = int(self.pause_time_entry.get())
        vel = float(self.velocity_entry.get())
        route_follow(nsteps, -stepval, pause, vel)




if __name__ == '__main__':
    root = tk.Tk()
    app = ScriptRunnerApp(root)
    root.mainloop()
