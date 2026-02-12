import serial
import time
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import matplotlib.pyplot as plt

GRIPPING_POINT = 0.057

class Imarobot:
    def __init__(self, model, connection_port, baudrate,
                 parameters_DH = DHRobot([                  # Создание робота по параметрам Денавита-Хартенберга
                    RevoluteDH(a=0, d=0.23682, alpha=np.pi / 2),  # Звено 1
                    RevoluteDH(a=0.32, d=0, alpha=0),  # Звено 2
                    RevoluteDH(a=0, d=0, alpha=np.pi / 2),  # Звено 3
                    RevoluteDH(a=0, d=0.2507, alpha=-np.pi / 2),  # Звено 4
                    RevoluteDH(a=0, d=0, alpha=np.pi / 2),  # Звено 5
                    RevoluteDH(a=0, d=GRIPPING_POINT, alpha=0)  # Звено 6
                ], name='ImaRobotStepMotors'),
                 old_range_min = np.array([180, -26.6, -79.5, 131.048, 95.972, -180]),
                 old_range_max = np.array([-180, 173.3, 130.5, -172.841, -116.755, 180]),
                 new_range_min = np.array([20000, 23997, 0, 4000, 0, 0]),
                 new_range_max = np.array([0, 0, 7000, 0, 2600, 7676])):
        self.model = model
        self.connection_port = connection_port
        self.baudrate = baudrate
        self.parameters_DH = parameters_DH
        self.old_range_min = old_range_min
        self.old_range_max = old_range_max
        self.new_range_min = new_range_min
        self.new_range_max = new_range_max


    def access_port(self):
        try:
            communication_port = serial.Serial(self.connection_port, self.baudrate, timeout=1)
            print(f"Порт {self.connection_port} открыт успешно.")
            return communication_port

        except serial.SerialException as e:
            print(f"Ошибка открытия порта {self.connection_port}: {e}")
            exit()

    def send_data_to_robot(self, x):

        print_flag = 1  # аналог print = 1

        # --- 4. Калибровочные диапазоны ---


        # --- 5. Задание углов и расчёт положения ---
        J_angles_in_deg = np.array([0, 147, -63, 0, 0, 0])  # Углы в градусах
        J_angles_in_rad = np.deg2rad(J_angles_in_deg)  # Перевод в радианы

        # --- 6. Преобразование углов и отправка в контроллер ---
        if print_flag == 1:
            Start = 255
            x.write(bytes([Start]))  # Отправка стартового байта (255)
            x.write(bytes([Start]))

            J_angles_out = np.zeros(6, dtype=int)
            left_8 = np.zeros(6, dtype=int)
            right_8 = np.zeros(6, dtype=int)

            for i in range(6):
                # Линейное преобразование угла
                J_angles_out[i] = int(
                    ((J_angles_in_deg[i] - ima_step_robot.old_range_min[i]) *
                     (ima_step_robot.new_range_max[i] - ima_step_robot.new_range_min[i]) /
                     (ima_step_robot.old_range_max[i] - ima_step_robot.old_range_min[i])) + ima_step_robot.new_range_min[i]
                )

                # Ограничение в рамках 16-битного беззнакового числа
                J_angles_out[i] = max(0, min(65535, J_angles_out[i]))

                # Преобразование в 16-битный бинарный вектор (старший бит слева)
                binVal = format(J_angles_out[i], '016b')  # строка из 16 бит

                # Разделение на два байта
                left_byte_str = binVal[:8]  # Первые 8 бит (старший байт)
                right_byte_str = binVal[8:]  # Последние 8 бит (младший байт)

                left_8[i] = int(left_byte_str, 2)
                right_8[i] = int(right_byte_str, 2)

                # Отправка байтов
                x.write(bytes([left_8[i]]))
                time.sleep(0.01)  # пауза 10 мс
                x.write(bytes([right_8[i]]))

            print("Данные отправлены в контроллер.")

        # --- 7. Закрытие порта ---
        x.close()
        print("Порт закрыт.")


        # Прямая кинематика (получение матрицы T)
        T = ima_step_robot.parameters_DH.fkine(J_angles_in_rad)
        print("Матрица однородного преобразования T:\n", T)

        # Визуализация ()
        fig = ima_step_robot.parameters_DH.plot(J_angles_in_rad, block=False)
        plt.ioff()  # Отключаем интерактивный режим
        plt.show()

if __name__ == "__main__":
    print("Запуск задачи")
    ima_step_robot = Imarobot ('ImaRobot', 'COM3', 9600)
    connection = ima_step_robot.access_port()
    ima_step_robot.send_data_to_robot(connection)
else:
    print("Не __main__")