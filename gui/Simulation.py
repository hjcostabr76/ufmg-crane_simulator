from typing import Tuple
from config import TIMEOUT_CONNECTION
import numpy as np
import math
from PyQt5 import QtGui
import matplotlib.pyplot as mpl

try:
    import sim
except:
    print ('Error import sim. Verify files and settings Coppelia.')

class Simulation():

    '''
        Gerencia conexao com 01 simulacao no coppelia sim.
    '''

    __client_id: int = None
    __is_connected: bool = False
    __is_running: bool = False


    def connect(self, ip: str = '127.0.0.1', port: int = 19997) -> Tuple[bool, int, str]:
        '''
            Conecta cliente ao coppelia.
            Retorna status da conexao, codigo da simulacao, descricao de falha

        '''

        self.__log('Trying to connect...')
        sim.simxFinish(-1) # just in case, close all opened connections
        self.__client_id = sim.simxStart(ip, port, True, True, TIMEOUT_CONNECTION, 5)
        self.__is_connected = self.__client_id != -1
        
        error = None
        if (self.__client_id == -1):
            self.__log('Connection could not be established')
        else:
            self.__log('Connection succesfuly stablished with simulation ' + str(self.__client_id) + ' at ' + ip + ':' + str(port))
            self.__log_coppelia('Group B connected...')

        return self.__is_connected, self.__client_id, error

    def disconnect(self) -> None:
        '''
            Desconect cliente do coppelia caso esteja conectado.
        '''

        if (not self.__is_connected):
            return
        
        self.stop()
        self.__log('Closing connection...')
        
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        # You can guarantee this with (for example):
        sim.simxGetPingTime(self.__client_id)
        sim.simxFinish(self.__client_id) # Now close the connection to CoppeliaSim:

        self.__is_connected = False
        self.__log('Connection closed')
    
    def is_simulation_running(self) -> bool:
        '''
            FIXME: 2021-08-23 - Fazer isso funcionar direito
        '''
        return sim.simxGetConnectionId(self.__client_id) != -1

    def start(self) -> None:
        '''
            Inicia simulacao caso haja conexao ativa.
        '''

        if (self.__is_connected):
            self.__log('Starting simulation...')
            sim.simxStartSimulation(self.__client_id, sim.simx_opmode_blocking)
            self.__log_coppelia('Group B have assumed controll...')
            self.__is_running = True

    def stop(self) -> None:
        '''
            Encerra simulacao caso esteja rodando.
        '''

        if (self.__is_running):
            self.__log('Stopping simulation...')
            sim.simxStopSimulation(self.__client_id, sim.simx_opmode_oneshot)
            self.__log_coppelia('Group B will be back...')
            self.__is_running = False

    def get_obj_handle(self, obj_name: str):
        '''
            Catch and return a simulation object handler by its name.
        '''

        return_code, handle = sim.simxGetObjectHandle(self.__client_id, obj_name, sim.simx_opmode_blocking)
        self.__validate_coppelia_return(return_code)
        return handle

    def debug_all_handles(self) -> None:
        
        foo = [
            (sim.sim_object_shape_type, 'Shapes'),
            (sim.sim_object_joint_type, 'Joints'),
            (sim.sim_object_camera_type, 'Cameras'),
            (sim.sim_object_proximitysensor_type, 'Proximity Sensors'),
            (sim.sim_object_visionsensor_type, 'Vision Sensors'),
            (sim.sim_object_forcesensor_type, 'Force Sensors'),
            (sim.sim_object_graph_type, 'Graphs'),
            (sim.sim_object_light_type, 'Lights'),
            (sim.sim_object_dummy_type, 'Dummies'),
            # sim_object_reserved1
            # sim_object_reserved2
            # sim_object_path_type
            # sim_object_volume_type
            # sim_object_mill_type
            # sim_object_mirror_type
        ]

        for bar in foo:
            self.__log('Handles [' + bar[1] + ']:')
            print(sim.simxGetObjects(self.__client_id, bar[0], sim.simx_opmode_blocking)[1])


    def get_joint_angular_displacement(self, obj_name: str) -> float:
        '''
            Return the angular displacement of a simulation joint in degrees.
        '''

        return math.degrees(self.__get_joint_displacement(obj_name))

    def get_joint_linear_displacement(self, obj_name: str) -> float:
        '''
            Return the linear displacement of a simulation joint in meters.
        '''
        
        return self.__get_joint_displacement(obj_name)

    def get_obj_position(self, obj_name: str) -> float:
        '''
            Return a simulation object absolute position coordinates in meters.
        '''

        return_code, position = sim.simxGetObjectPosition(self.__client_id, self.get_obj_handle(obj_name), -1, sim.simx_opmode_blocking)
        self.__validate_coppelia_return(return_code)
        return position

    def run_script(self, obj_name: str, func_name: str):
        sim.simxCallScriptFunction(self.__client_id, obj_name, sim.sim_scripttype_childscript, func_name, [], [], [], bytearray(), sim.simx_opmode_blocking)

    def set_joint_velocity(self, obj_name: str, velocity: float) -> None:
        sim.simxSetJointTargetVelocity(self.__client_id, self.get_obj_handle(obj_name), velocity, sim.simx_opmode_oneshot)

    def force_sensor_init(self, sensor_obj_name: str) -> None:
        '''
            Initialize a force sensor so it can be read later.
            First calling should be in streaming mode and others in buffer mode (according to the docs).
        '''

        return_code, _, _, _ = sim.simxReadForceSensor(self.__client_id, self.get_obj_handle(sensor_obj_name), sim.simx_opmode_streaming)
        self.__validate_coppelia_return(return_code, False)

    def get_force_sensor_force(self, sensor_obj_name: str) -> Tuple:
        '''
            Read a force sensor and return if it is broken and its force vector reading.
            TODO: 2021-08-27 - Reajustar validacao
        '''

        return_code, state, force_vec, _ = sim.simxReadForceSensor(self.__client_id, self.get_obj_handle(sensor_obj_name), sim.simx_opmode_streaming)
        self.__validate_coppelia_return(return_code, False)
        is_broken = state > 1
        return is_broken, force_vec

    def prox_sensor_init(self, sensor_obj_name: str, distance_init_obj_name: str = None) -> None:
        '''
            TODO: 2021-08-30 - Atualizar descricao
            Initialize a proximity sensor so it can be read later.
            First calling should be in streaming mode and others in buffer mode (according to the docs).
        '''
        
        sensor_handle = self.get_obj_handle(sensor_obj_name)
        return_code1, _, _, _, _ = sim.simxReadProximitySensor(self.__client_id, sensor_handle, sim.simx_opmode_streaming)
        self.__validate_coppelia_return(return_code1, False)

        if distance_init_obj_name:
            return_code2, _ = sim.simxCheckDistance(self.__client_id, sensor_handle, self.get_obj_handle(distance_init_obj_name), sim.simx_opmode_streaming)
            # print('obj distance init...', return_code2, _)
            self.__validate_coppelia_return(return_code2, False)

    def get_prox_sensor_distance(self, obj_name: str) -> Tuple[Tuple, None]:
        '''
            TODO: 2021-08-24 - ADD Descricao
            TODO: 2021-08-24 - Reajustar validacao
            TODO: 2021-08-24 - Concluir implementacao
        '''

        sensor_handle = self.get_obj_handle(obj_name)
        return_code, have_detected, detected_point, detected_obj, _ = sim.simxReadProximitySensor(self.__client_id, sensor_handle, sim.simx_opmode_buffer)
        self.__validate_coppelia_return(return_code, False)
        # print('prox sensor:', return_code, have_detected, detected_point, detected_obj)
        if have_detected:
            return detected_obj, self.get_distance_between_objects(sensor_handle, detected_obj)

    def get_distance_between_objects(self, obj1: int, obj2: int) -> float:
        '''
            TODO: 2021-08-30 - ADD Descricao
        '''

        return_code, min_distance = sim.simxCheckDistance(self.__client_id, obj1, obj2, sim.simx_opmode_buffer)
        self.__validate_coppelia_return(return_code)
        return min_distance

    def vision_sensor_init(self, sensor_obj_name: str) -> None:
        '''
            Initialize a vision sensor so it can be read later.
            First calling should be in streaming mode and others in buffer mode (according to the docs).
        '''

        return_code, _, _ = sim.simxGetVisionSensorImage(self.__client_id, self.get_obj_handle(sensor_obj_name), 0, sim.simx_opmode_streaming)
        self.__validate_coppelia_return(return_code, False)

    def get_vision_sensor_img_matrix(self, sensor_obj_name: str) -> Tuple[np.array, None]:
        '''
            Generates and return an image rgb bits matrix after reading a vision sensor.
            It expects that the vision sensor is already initialized.
            TODO: 2021-08-27 - Reajustar validacao
        '''

        return_code, resolution, image = sim.simxGetVisionSensorImage(self.__client_id, self.get_obj_handle(sensor_obj_name), 0, sim.simx_opmode_buffer)
        self.__validate_coppelia_return(return_code, False)
        
        if len(resolution):
            height, width = resolution
            img = np.array(image, dtype = np.uint8)
            img.resize([height, width, 3])
            img = np.array(img[::-1], dtype=np.uint8)
            return img

    def get_vision_sensor_img(self, sensor_obj_name: str, ) -> Tuple[QtGui.QImage, None]:
        '''
            Generates and return an image after reading a vision sensor.
            It expects that the vision sensor is already initialized.
        '''

        img = self.get_vision_sensor_img_matrix(sensor_obj_name)
        if isinstance(img, np.ndarray):
            height, width, _ = img.shape
            bytesPerLine = 3 * width # TODO: Understand why we use this bytes quantity
            return QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888)

    def print_vision_sensor_img(self, sensor_obj_name: str) -> None:
        '''
            Generates and prints an image after reading a vision sensor.
            It expects that the vision sensor is already initialized.
        '''

        img = self.get_vision_sensor_img_matrix(sensor_obj_name)
        if isinstance(img, np.ndarray):
            mpl.imshow(img, origin='lower')
            mpl.show()

    def save_vision_sensor_img(self, sensor_obj_name: str, file_name: str = None) -> QtGui.QImage:
        '''
            Save an image after reading a vision sensor.
            It expects that the vision sensor is already initialized.
        '''

        img = self.get_vision_sensor_img_matrix(sensor_obj_name)
        if isinstance(img, np.ndarray):
            mpl.imshow(img, origin='lower')
            mpl.savefig(file_name)

    def __get_joint_displacement(self, obj_name: str) -> float:
        '''
            Return the intrinsic displacement of a simulation object:
            It might be radians or metres depending on the type of the joint;
        '''

        return_code, displacement = sim.simxGetJointPosition(self.__client_id, self.get_obj_handle(obj_name), sim.simx_opmode_blocking)
        self.__validate_coppelia_return(return_code)
        return displacement

    def __validate_coppelia_return(self, return_code: int, should_throw_on_no_value: bool = True) -> None:
        '''
            Avaliar o codigo de retorno de 01 operacao do coppelia. Notifica & lanca excessao em caso de falha.
            See: https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
        '''

        if (return_code == sim.simx_return_ok):
            return
        
        return_name: str = None

        if (sim.simx_return_novalue_flag == sim.simx_return_novalue_flag):
            if not should_throw_on_no_value:
                return
            return_name = 'simx_return_novalue_flag'

        if (sim.simx_return_novalue_flag == sim.simx_return_novalue_flag):
            return_name = 'simx_return_novalue_flag'
        elif (sim.simx_return_timeout_flag == sim.simx_return_timeout_flag):
            return_name = 'simx_return_timeout_flag'
        elif (sim.simx_return_illegal_opmode_flag == sim.simx_return_illegal_opmode_flag):
            return_name = 'simx_return_illegal_opmode_flag'
        elif (sim.simx_return_remote_error_flag == sim.simx_return_remote_error_flag):
            return_name = 'simx_return_remote_error_flag'
        elif (sim.simx_return_split_progress_flag == sim.simx_return_split_progress_flag):
            return_name = 'simx_return_split_progress_flag'
        elif (sim.simx_return_local_error_flag == sim.simx_return_local_error_flag):
            return_name = 'simx_return_local_error_flag'
        elif (sim.simx_return_initialize_error_flag == sim.simx_return_initialize_error_flag):
            return_name = 'simx_return_initialize_error_flag'
        
        raise Exception('Bad operation return code: ' + str(return_code) + ' [' + return_name + ']')

    def __log(self, msg: str) -> None:
        print('[sim]', msg)

    def __log_coppelia(self, msg: str):
        if (self.__is_connected):
            sim.simxAddStatusbarMessage(self.__client_id, msg, sim.simx_opmode_oneshot)
