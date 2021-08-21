from ctypes import ArgumentError
from Simulation import Simulation
from common import *
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
        self.__set_obj_velocity(SIM_ARM, velocity)
        pass

    def set_crab_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(SIM_CRAB, velocity)
        pass

    def set_hoist_vertical_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(SIM_HOIST_VERTICAL, velocity)
        pass

    def set_hoist_angular_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(SIM_HOIST_ANGULAR, velocity)
        pass

    def toggle_magnet_state(self) -> bool:
        self.__simulation.run_script(SIM_BASE, SIM_MAGNET_SCRIPT)
        self.__is_magnet_active = not self.__is_magnet_active
        return self.__is_magnet_active

    def get_arm_angle(self) -> float:
        angle = self.__simulation.get_joint_angle(SIM_ARM)
        return angle

    def __set_obj_velocity(self, obj_name: str, velocity: int) -> None:
        if (type(velocity) != int or math.fabs(velocity) > VELOCITY_MAX):
            raise ArgumentError('Velocity must be an integer from -' + str(VELOCITY_MAX) + ' to ' + str(VELOCITY_MAX) + ' (' + str(velocity) + ' received)')
        self.__simulation.set_joint_velocity(obj_name, velocity / 100)

    def __log(self, msg: str) -> None:
        print('[controller]', msg)
