#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Калибровка двух роботов-манипуляторов UR5e
Использует: urx, math3d, numpy, scipy
"""

import urx
import math3d as m3d
from urx_compat import patch_urx_math3d
patch_urx_math3d()
import numpy as np
import time
from calibration_manager import CalibrationManager
from config import load_config


class DualRobotSystem:
    def __init__(self, ip1, ip2, calib_file="calibration_data.json", tcp=None):
        self.rob1 = urx.Robot(ip1)
        self.rob2 = urx.Robot(ip2)
        self.calib = CalibrationManager(calib_file)
        print(f"✓ Подключено к Роботу 1: {ip1}")
        print(f"✓ Подключено к Роботу 2: {ip2}")

        self.rob1.set_tcp(tcp)
        self.rob2.set_tcp(tcp)

    def check_collision_risk(self, pose1, pose2, min_distance=0.2):
        """
        Проверка риска столкновения между роботами.

        Args:
            pose1: поза Робота 1 [x,y,z, ...]
            pose2: поза Робота 2 [x,y,z, ...]
            min_distance: минимальное безопасное расстояние между базами роботов (м)

        Returns:
            bool: True если безопасно, False если риск столкновения
        """
        pos1 = np.array(pose1[:3])
        pos2 = np.array(pose2[:3])

        distance = np.linalg.norm(pos1 - pos2)

        # Расстояние между TCP роботов
        tcp_distance = distance

        print(f"  Расстояние между TCP: {tcp_distance * 1000:.1f} мм")

        if tcp_distance < min_distance:
            print(f"  ⚠ ВНИМАНИЕ: Роботы слишком близко! (< {min_distance * 1000:.0f} мм)")
            return False

        return True
    def collect_points(self, n_points=5):
        """
        Интерактивный сбор калибровочных точек.
        Оператор вручную подводит роботов к общей точке.
        """
        positions1 = []
        positions2 = []

        print(f"\n{'='*50}")
        print(f"СБОР {n_points} КАЛИБРОВОЧНЫХ ТОЧЕК")
        print(f"{'='*50}")
        print("Инструкция:")
        print("1. Вручную подведите Робота 1 к реперной точке")
        print("2. Нажмите Enter в консоли")
        print("3. Вручную подведите Робота 2 к ТОЙ ЖЕ точке")
        print("4. Нажмите Enter в консоли")
        print("5. Повторите для всех точек\n")

        for i in range(n_points):
            self.rob1.set_freedrive(True)
            input(f"Точка {i+1}/{n_points}: Робот 1 готов? (Enter)")
            self.rob1.set_freedrive(False)
            pose1 = self.rob1.getl()
            positions1.append(m3d.Vector(pose1[:3]))
            print(f"  Записано: [{pose1[0]:.3f}, {pose1[1]:.3f}, {pose1[2]:.3f}] м")

            self.rob2.set_freedrive(True)
            input(f"Точка {i+1}/{n_points}: Робот 2 готов? (Enter)")
            self.rob2.set_freedrive(False)
            pose2 = self.rob2.getl()
            positions2.append(m3d.Vector(pose2[:3]))
            print(f"  Записано: [{pose2[0]:.3f}, {pose2[1]:.3f}, {pose2[2]:.3f}] м")
            print()

        return positions1, positions2

    def run_calibration(self, n_points=5):
        """Запуск процесса калибровки"""
        print("\n⚠ НАЧАЛО КАЛИБРОВКИ")
        print("Убедитесь, что роботы в безопасном режиме!\n")

        # Сбор точек
        pts1, pts2 = self.collect_points(n_points)

        # Вычисление трансформации
        print("Вычисление трансформации...")
        self.calib.calculate_transform_from_points(pts1, pts2)

        # Проверка
        if self.calib.verify():
            print("✓ Матрица поворота корректна")
        else:
            print("⚠ Внимание: матрица поворота может быть некорректной!")

        # Сохранение
        self.calib.save(metadata={
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "n_points": n_points,
            "robots": "UR5e x2"
        })

        # Информация
        dist = self.calib.get_translation_distance()
        print(f"\n{'='*50}")
        print(f"РЕЗУЛЬТАТЫ КАЛИБРОВКИ")
        print(f"{'='*50}")
        print(f"Расстояние между базами: {dist*1000:.1f} мм")
        print(f"Вектор переноса: {self.calib.transform.pos.get_array()} м")
        print(f"{'='*50}\n")

    def test_synchronization(self):
        """Тест синхронизации поз двух роботов"""
        if self.calib.transform is None:
            print("⚠ Калибровка не загружена!")
            return

        print("\n{'='*50}")
        print("ТЕСТ СИНХРОНИЗАЦИИ")
        print(f"{'='*50}")

        # Получаем позу Робота 1
        self.rob1.set_freedrive(True)
        input("Подведите Робота 1 в тестовую точку и нажмите Enter...")
        self.rob1.set_freedrive(False)
        pose1 = self.rob1.getl()
        print(f"Поза Робота 1: {pose1[:3]} м")

        # Вычисляем позу для Робота 2
        pose2 = self.calib.transform_pose(pose1)
        print(f"Целевая поза Робота 2: {pose2[:3]} м")

        # Предложение двигать Робота 2
        confirm = input("Отправить Робота 2 в эту точку? (y/n): ")
        if confirm.lower() == 'y':
            print("Движение Робота 2...")
            self.rob2.movel(pose2, acc=0.3, vel=0.15)
            print("✓ Робот 2 достиг целевой точки")
        else:
            print("Движение отменено")

    def test_safe_synchronization(self, orientation_offset=None):
        """
        БЕЗОПАСНЫЙ тест синхронизации.
        Робот 2 подводит TCP в ту же точку, но с другой ориентацией.

        Args:
            orientation_offset: [rx, ry, rz] в радианах
                               По умолчанию [0, 0, π] (разворот на 180°)
        """
        if self.calib.transform is None:
            print("⚠ Калибровка не загружена!")
            return

        print(f"\n{'=' * 60}")
        print("БЕЗОПАСНЫЙ ТЕСТ СИНХРОНИЗАЦИИ")
        print(f"{'=' * 60}")
        print("Робот 2 подведёт TCP в ту же точку, но с другой ориентацией")
        print("Это предотвратит столкновение роботов\n")

        # Получаем позу Робота 1
        self.rob1.set_freedrive(True, 120)
        input("1. Подведите Робота 1 в тестовую точку и нажмите Enter...")
        self.rob1.set_freedrive(False)
        pose1 = self.rob1.getl()
        print(f"   Поза Робота 1: {pose1[:3]} м")
        print(f"   Ориентация Робота 1: {pose1[3:]} рад")

        # Вычисляем безопасную позу для Робота 2
        pose2 = self.calib.transform_position_only(pose1, orientation_offset)
        print(f"\n   Целевая поза Робота 2: {pose2[:3]} м")
        print(f"   Ориентация Робота 2: {pose2[3:]} рад")

        # Проверка на столкновение
        print("\n   Проверка безопасности...")
        if not self.check_collision_risk(pose1, pose2, min_distance=0.05):
            print("\n   ✗ Движение отменено из-за риска столкновения!")
            return

        # Предложение двигать Робота 2
        confirm = input("\n   Отправить Робота 2 в эту точку? (y/n): ")
        if confirm.lower() == 'y':
            print("   Движение Робота 2...")
            self.rob2.movel(pose2, acc=0.3, vel=0.15)
            print("   ✓ Робот 2 достиг целевой точки")

            # Проверка точности совпадения TCP
            pose2_actual = self.rob2.getl()
            tcp_error = np.linalg.norm(np.array(pose2[:3]) - np.array(pose2_actual[:3]))
            print(f"   Точность позиционирования: {tcp_error * 1000:.2f} мм")
        else:
            print("   Движение отменено")

    def test_approach_from_opposite(self):
        """
        Тест: Роботы подходят к одной точке с противоположных сторон.
        Идеально для передачи детали.
        """
        if self.calib.transform is None:
            print("⚠ Калибровка не загружена!")
            return

        print(f"\n{'=' * 60}")
        print("ТЕСТ: ПОДХОД С ПРОТИВОПОЛОЖНЫХ СТОРОН")
        print(f"{'=' * 60}")
        print("Робот 2 развернётся на 180° относительно Робота 1")
        print("Это позволит передавать деталь без столкновения\n")

        # Ориентация: разворот на 180° вокруг Z
        orientation_offset = [0, 0, np.pi/2]

        self.test_safe_synchronization(orientation_offset)

    def test_approach_from_above(self):
        """
        Тест: Робот 2 подходит сверху (вертикально).
        """
        if self.calib.transform is None:
            print("⚠ Калибровка не загружена!")
            return

        print(f"\n{'=' * 60}")
        print("ТЕСТ: ВЕРТИКАЛЬНЫЙ ПОДХОД")
        print(f"{'=' * 60}")
        print("Робот 2 подойдёт к точке вертикально сверху\n")

        # Ориентация: вертикально вниз (разворот на 180° вокруг X)
        orientation_offset = [np.pi, 0, 0]

        self.test_safe_synchronization(orientation_offset)

    def run(self):
        """Основной цикл программы"""
        print("\n" + "="*50)
        print("КАЛИБРОВКА ДВУХ РОБОТОВ UR5e")
        print("="*50)

        # Попытка загрузить существующую калибровку
        try:
            self.calib.load()
            if self.calib.verify():
                print("✓ Калибровка загружена и валидна")
                dist = self.calib.get_translation_distance()
                print(f"  Расстояние между базами: {dist*1000:.1f} мм")
            else:
                print("⚠ Загруженная калибровка некорректна")
                raise ValueError("Bad calibration")
        except (FileNotFoundError, ValueError) as e:
            print(f"! Требуется новая калибровка: {e}")
            self.run_calibration(n_points=5)

        # Меню
        while True:
            print("\n" + "=" * 60)
            print("МЕНЮ")
            print("=" * 60)
            print("1. Тест: Безопасная синхронизация (разворот 180°)")
            print("2. Тест: Вертикальный подход сверху")
            print("3. Тест: Полная трансформация позы (⚠ опасно!)")
            print("4. Новая калибровка")
            print("5. Показать параметры калибровки")
            print("6. Выход")

            choice = input("\nВыберите опцию (1-6): ")

            if choice == '1':
                self.test_approach_from_opposite()
            elif choice == '2':
                self.test_approach_from_above()
            elif choice == '3':
                print("\n⚠ ВНИМАНИЕ: Полная трансформация может привести к столкновению!")
                confirm = input("Продолжить? (y/n): ")
                if confirm.lower() == 'y':
                    self.test_synchronization_full()
            elif choice == '4':
                self.run_calibration(n_points=5)
            elif choice == '5':
                if self.calib.transform:
                    print(f"\nВектор переноса: {self.calib.transform.pos.get_array()} м")
                    print(f"Расстояние: {self.calib.get_translation_distance() * 1000:.1f} мм")
                else:
                    print("Калибровка не загружена")
            elif choice == '6':
                print("Выход...")
                break
            else:
                print("Неверный выбор")

    def close(self):
        """Закрытие подключений"""
        self.rob1.close()
        self.rob2.close()
        print("✓ Подключения закрыты")


# ============================================================================
# ТОЧКА ВХОДА
# ============================================================================

if __name__ == "__main__":
    # IP-адреса ваших роботов
    cfg = load_config()
    IP_ROBOT_1 = cfg.diagnost_ip
    IP_ROBOT_2 = cfg.surgeon_ip
    TCP_TOOL = [0,0,0.125,0,0,0]
    system = None

    try:
        system = DualRobotSystem(IP_ROBOT_1, IP_ROBOT_2, tcp=TCP_TOOL)
        system.run()
    except KeyboardInterrupt:
        print("\n⚠ Прервано пользователем")
    except Exception as e:
        print(f"\n✗ Ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if system:
            system.close()
