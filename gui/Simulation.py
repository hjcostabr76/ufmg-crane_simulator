from typing import Tuple
from common import TIMEOUT
import math

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
        self.__client_id = sim.simxStart(ip, port, True, True, TIMEOUT, 5)
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
            op_result = sim.simxStopSimulation(self.__client_id, sim.simx_opmode_oneshot)
            self.__log_coppelia('Group B will be back...')
            self.__is_running = False

    def get_obj_handle(self, obj_name: str):
        '''
            Catch and return a simulation object handler by its name.
        '''
        handle_result = sim.simxGetObjectHandle(self.__client_id, obj_name, sim.simx_opmode_blocking)
        self.__validate_coppelia_return(handle_result[0])
        return handle_result[1]

    def get_joint_angle(self, obj_name: str) -> float:
        '''
            Return the intrinsic angle of a simulation object in degrees.
        '''

        result = sim.simxGetJointPosition(self.__client_id, self.get_obj_handle(obj_name), sim.simx_opmode_blocking)
        self.__validate_coppelia_return(result[0])
        return math.degrees(result[1])

    def run_script(self, obj: str, func: str):
        sim.simxCallScriptFunction(self.__client_id, obj, sim.sim_scripttype_childscript, func, [], [], [], bytearray(), sim.simx_opmode_blocking)

    def set_joint_velocity(self, obj_name: str, velocity: float) -> None:
        sim.simxSetJointTargetVelocity(self.__client_id, self.get_obj_handle(obj_name), velocity, sim.simx_opmode_oneshot)

    def __validate_coppelia_return(self, return_code: int) -> None:
        '''
            Avaliar o codigo de retorno de 01 operacao do coppelia. Notifica & lanca excessao em caso de falha.
            See: https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
        '''

        if (return_code == sim.simx_return_ok):
            return
        
        return_name: str = None

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
        
        raise NotImplementedError('Bad operation return code: ' + str(return_code) + '[' + return_name + ']')

    def __log(self, msg: str) -> None:
        print('[sim]', msg)

    def __log_coppelia(self, msg: str):
        if (self.__is_connected):
            sim.simxAddStatusbarMessage(self.__client_id, msg, sim.simx_opmode_oneshot)
