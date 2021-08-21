from typing import Tuple
from common import TIMEOUT

try:
    import sim
except:
    print ('Error import sim. Verify files and settings Coppelia.')

class Simulation():

    '''
        Gerencia conexao com 01 simulacao no coppelia sim.
    '''

    sim_id: int = None
    is_connected: bool = False
    is_running: bool = False

    def connect(self, ip: str = '127.0.0.1', port: int = 19997) -> Tuple[bool, int, str]:
        '''
            Conecta cliente ao coppelia.
            Retorna status da conexao, codigo da simulacao, descricao de falha

        '''

        self.log('Trying to connect...')
        sim.simxFinish(-1) # just in case, close all opened connections
        self.sim_id = sim.simxStart(ip, port, True, True, TIMEOUT, 5)
        self.is_connected = self.sim_id != -1
        
        error = None
        if (self.sim_id == -1):
            self.log('Connection could not be established')
        else:
            self.log('Connection succesfuly stablished with simulation ' + str(self.sim_id) + ' at ' + ip + ':' + str(port))
            self.log_coppelia('Group B connected...')

        return self.is_connected, self.sim_id, error

    def disconnect(self) -> None:
        '''
            Desconect cliente do coppelia caso esteja conectado.

        '''

        if (not self.is_connected):
            return
        
        self.stop()
        self.log('Closing connection...')
        
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        # You can guarantee this with (for example):
        sim.simxGetPingTime(self.sim_id)
        sim.simxFinish(self.sim_id) # Now close the connection to CoppeliaSim:

        self.is_connected = False
        self.log('Connection closed')

    def start(self) -> None:
        '''
            Inicia simulacao caso haja conexao ativa.

        '''

        if (self.is_connected):
            self.log('Starting simulation...')
            sim.simxStartSimulation(self.sim_id, sim.simx_opmode_blocking)
            self.log_coppelia('Group B have assumed controll...')
            self.is_running = True

    def stop(self) -> None:
        '''
            Encerra simulacao caso esteja rodando.

        '''

        if (self.is_running):
            self.log('Stopping simulation...')
            op_result = sim.simxStopSimulation(self.sim_id, sim.simx_opmode_oneshot)
            self.log_coppelia('Group B will be back...')
            self.is_running = False

    def get_obj_handle(self, obj_name: str):
        handle_result = sim.simxGetObjectHandle(self.sim_id, obj_name, sim.simx_opmode_blocking)
        self.validate_coppelia_return(handle_result[0])
        return handle_result[1]

    def run_script(self, obj: str, func: str):
        sim.simxCallScriptFunction(self.sim_id, obj, sim.sim_scripttype_childscript, func, [], [], [], bytearray(), sim.simx_opmode_blocking)

    def set_joint_velocity(self, handler, velocity: float) -> None:
        sim.simxSetJointTargetVelocity(self.sim_id, handler, velocity, sim.simx_opmode_oneshot)

    def validate_coppelia_return(self, return_code: int) -> None:

        '''
            TODO: 2021-08-20 - Concluir implementacao
            See: https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
        '''

        if (return_code == sim.simx_return_ok):
            return

        # sim.simx_return_novalue_flag
        # sim.simx_return_timeout_flag
        # sim.simx_return_illegal_opmode_flag
        # sim.simx_return_remote_error_flag
        # sim.simx_return_split_progress_flag
        # sim.simx_return_local_error_flag
        # sim.simx_return_initialize_error_flag

        raise NotImplementedError()

    def log(self, msg: str) -> None:
        print('[sim]', msg)

    def log_coppelia(self, msg: str):
        if (self.is_connected):
            sim.simxAddStatusbarMessage(self.sim_id, msg, sim.simx_opmode_oneshot)
