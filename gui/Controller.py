from ctypes import ArgumentError
from Simulation import Simulation

try:
    import sim
except:
    print ('Error import sim. Verify files and settings Coppelia.')


''' ==========================================================
-- CONSTANTES ------------------------------------------------
========================================================== '''

VELOCITY_MIN = -10
VELOCITY_MAX = 10

# Objetos da simulacao
SIM_ARM = 'Arm_actuator' # Lanca
SIM_BASE = 'Base_PS' # Base do guindaste
SIM_CRAB = 'Crab_actuator' # Guincho
SIM_HOIST_ANGULAR = 'Revolute_hoist' # Garra: Junta de movimento rotacional
SIM_HOIST_VERTICAL = 'Hoist_actuator' # Garra: Junta de monvimento vertical

SIM_MAGNET = 'suctionPad' # Ima
SIM_MAGNET_SCRIPT = 'actuateMagnetRemote' # Ima: Funcao que atualiza seu estado

SIM_CAM1 = 'Camera_Panoramica' # Camera panoramica
SIM_CAM2 = 'Camera_Guincho' # Camera do guincho

SIM_SENS_FORCE_CABIN = 'Force_sensor' # Sensor de forca na cabine
SIM_SENS_FORCE_CRAB = 'Force_sensor0' # Sensor de forca no guincho
SIM_SENS_VISION = 'Vision_sensor' # Sensor de visao no guincho
SIM_SENS_PROX = 'Proximity_sensor' # Sensor de proximidade no guincho

class Controller:

    '''
        Gerencia acoes de controle do guindates durante uma simulacao ativa.
    '''

    __simulation: Simulation = None

    __arm = None
    __crab = None
    __hoist_vertical = None
    __hoist_angular = None
    # __magnet = None

    __is_magnet_active: bool = False

    # cam1 = None
    # cam2 = None
    # sensor_force_cabin = None
    # sensor_force_crab = None
    # sensor_vision = None
    # sensor_prox = None


    def __init__(self, simulation: Simulation) -> None:

        self.__simulation = simulation

        # Captura handlers dos objetos da simulacao
        self.__arm = self.__simulation.get_obj_handle(SIM_ARM)
        self.__crab = self.__simulation.get_obj_handle(SIM_CRAB)
        self.__hoist_vertical = self.__simulation.get_obj_handle(SIM_HOIST_VERTICAL)
        self.__hoist_angular = self.__simulation.get_obj_handle(SIM_HOIST_ANGULAR)
        # self.__magnet = self.__simulation.get_obj_handle(SIM_MAGNET)
        # self.cam1 = self.get_object_handle(SIM_CAM1)
        # self.cam2 = self.get_object_handle(SIM_CAM2)
        # self.sensor_force_cabin = self.get_object_handle(SIM_SENS_FORCE_CABIN)
        # self.sensor_force_crab = self.get_object_handle(SIM_SENS_FORCE_CRAB)
        # self.sensor_vision = self.get_object_handle(SIM_SENS_VISION)
        # self.sensor_prox = self.get_object_handle(SIM_SENS_PROX)

    def set_arm_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(self.__arm, velocity)
        pass

    def set_crab_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(self.__crab, velocity)
        pass

    def set_hoist_vertical_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(self.__hoist_vertical, velocity)
        pass

    def set_hoist_angular_vel(self, velocity: int) -> None:
        self.__set_obj_velocity(self.__hoist_angular, velocity)
        pass

    def toggle_magnet_state(self) -> bool:
        self.__simulation.run_script(SIM_BASE, SIM_MAGNET_SCRIPT)
        self.__is_magnet_active = not self.__is_magnet_active
        return self.__is_magnet_active

    def __set_obj_velocity(self, handler, velocity: int) -> None:
        if (type(velocity) != int or velocity < VELOCITY_MIN or velocity > VELOCITY_MAX):
            raise ArgumentError('Velocity must be an integer from 0 to 10 (' + str(velocity) + ' received)')
        self.__simulation.set_joint_velocity(handler, velocity / 100)

    def __log(self, msg: str) -> None:
        print('[controller]', msg)
