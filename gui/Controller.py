from ctypes import ArgumentError
from Simulation import Simulation
from config import *
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

    def set_arm_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(SIM_ARM_JOINT, velocity)

    def set_crab_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(SIM_CRAB_JOINT, velocity)

    def set_hoist_vertical_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(SIM_HOIST_JOINT_VERTICAL, velocity)

    def set_hoist_angular_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(SIM_HOIST_JOINT_ANGULAR, velocity)

    def toggle_magnet_state(self) -> bool:
        self.__simulation.run_script(SIM_BASE, SIM_MAGNET_SCRIPT)
        self.__is_magnet_active = not self.__is_magnet_active
        return self.is_magnet_active()

    def get_arm_angle(self) -> float:
        return self.__simulation.get_joint_angular_displacement(SIM_ARM_JOINT)

    def get_crab_position(self) -> float:
        return self.__get_parsed_distance(self.__simulation.get_obj_position(SIM_CRAB)[0])

    def get_hoist_height(self) -> float:
        return self.__get_parsed_distance(self.__simulation.get_obj_position(SIM_HOIST)[-1])

    def get_hoist_angle(self) -> float:
        return self.__simulation.get_joint_angular_displacement(SIM_HOIST_JOINT_ANGULAR)

    def is_magnet_active(self) -> float:
        return self.__is_magnet_active

    def get_proximity_sensor_reading(self) -> None:
        self.__simulation.get_sensor_reading(SIM_SENS_PROX)

    def __get_parsed_distance(self, distance: float) -> float:
        return SCALE * distance

    def __set_obj_velocity(self, obj_name: str, velocity: int) -> None:
        if (type(velocity) != int or math.fabs(velocity) > VELOCITY_MAX):
            raise ArgumentError('Velocity must be an integer from -' + str(VELOCITY_MAX) + ' to ' + str(VELOCITY_MAX) + ' (' + str(velocity) + ' received)')
        self.__simulation.set_joint_velocity(obj_name, velocity / 100)

    def __log(self, msg: str) -> None:
        print('[controller]', msg)
