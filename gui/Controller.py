from ctypes import ArgumentError
from Simulation import Simulation
from config import *
from PyQt5 import QtGui
import math

try:
    import sim
except:
    print ('Error import sim. Verify files and settings Coppelia.')

class Controller:

    '''
        Gerencia acoes de controle do guindates durante uma simulacao ativa.
    '''

    __simulation: Simulation = None
    __is_magnet_active: bool = False

    def __init__(self, simulation: Simulation) -> None:
        self.__simulation = simulation
        simulation.vision_sensor_init(SIM_CAM2)

    def start_simulation(self) -> None:
        self.__simulation.start()
    
    def stop_simulation(self) -> None:
        self.__simulation.stop()

    def set_arm_vel(self, level: int) -> None:
        # Velocidade máxima pra girar a lança: 0.6 rad/s
        self.__set_obj_velocity(SIM_ARM_JOINT, -level, .4)

    def set_crab_vel(self, level: int) -> None:
        # Velocidade máxima pra movimentar o guincho pra frente e pra trás: 1.13 m/s
        self.__set_obj_velocity(SIM_CRAB_JOINT, level, 1)

    def set_hoist_vertical_vel(self, level: int) -> None:
        # Velocidade máxima pra subir e abaixar o guincho: 1.35 m/s
        self.__set_obj_velocity(SIM_HOIST_JOINT_VERTICAL, level, 1)

    def set_hoist_angular_vel(self, level: int) -> None:
        # Velocidade máxima pra girar o guincho: 0.8 rad/s
        self.__set_obj_velocity(SIM_HOIST_JOINT_ANGULAR, -level, .8)

    def toggle_magnet_state(self) -> bool:
        self.__simulation.run_script(SIM_BASE, SIM_MAGNET_SCRIPT)
        self.__is_magnet_active = not self.__is_magnet_active
        return self.is_magnet_active()

    def turn_magnet_on(self) -> None:
        if not self.__is_magnet_active:
            self.toggle_magnet_state()
    
    def turn_magnet_off(self) -> None:
        if self.__is_magnet_active:
            self.toggle_magnet_state()

    def is_magnet_active(self) -> bool:
        return self.__is_magnet_active

    def get_arm_angle(self) -> float:
        return self.__simulation.get_joint_angular_displacement(SIM_ARM_JOINT)

    def get_crab_position(self) -> float:
        return self.__get_parsed_position(self.__simulation.get_obj_position(SIM_CRAB)[0])

    def get_hoist_height(self) -> float:
        return self.__get_parsed_position(self.__simulation.get_obj_position(SIM_HOIST)[-1])

    def get_hoist_angle(self) -> float:
        return self.__simulation.get_joint_angular_displacement(SIM_HOIST_JOINT_ANGULAR)

    def get_proximity_sensor_reading(self) -> None:
        self.__simulation.get_sensor_reading_proximity(SIM_SENS_PROX)

    def get_cam_img(self, cam_number: int) -> QtGui.QImage:
        if (cam_number not in [1, 2]):
            raise ValueError('Camera number should be 1 or 2. [' + str(cam_number) + ' received]')
        return self.__simulation.get_vision_sensor_img(SIM_CAM1 if (cam_number == 1) else SIM_CAM2)

    def is_load_attached(self) -> bool:
        '''
            TODO: 2021-08-27 - ADD Descricao
        '''
        return False

    def __get_parsed_position(self, distance: float) -> float:
        return SCALE * distance

    def __set_obj_velocity(self, obj_name: str, level: int, max_vel: float) -> None: 
        if (type(level) != int or math.fabs(level) > VELOCITY_MAX_LEVEL):
            raise ArgumentError('Velocity must be an integer from -' + str(VELOCITY_MAX_LEVEL) + ' to ' + str(VELOCITY_MAX_LEVEL) + ' (' + str(level) + ' received)')
        self.__simulation.set_joint_velocity(obj_name, max_vel*level / VELOCITY_MAX_LEVEL)

    def __log(self, msg: str) -> None:
        print('[controller]', msg)
