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

    clientID: int = None
    is_connected: bool = False
    is_running: bool = False

    def connect(self, ip: str = '127.0.0.1', port: int = 19997) -> Tuple[bool, int, str]:
        '''
            Conecta cliente ao coppelia.
            Retorna status da conexao, codigo da simulacao, descricao de falha

        '''

        self.log('Trying to connect...')
        sim.simxFinish(-1) # just in case, close all opened connections
        self.clientID = sim.simxStart(ip, port, True, True, TIMEOUT, 5)
        self.is_connected = self.clientID != -1
        
        error = None
        if (self.clientID == -1):
            self.log('Connection could not be established')
            error = sim.simxGetLastErrors()[0]
        else:
            self.log('Connection succesfuly stablished with simulation ' + str(self.clientID) + ' at ' + ip + ':' + str(port))
            sim.simxAddStatusbarMessage(self.clientID, 'Group B connected...', sim.simx_opmode_oneshot)

        return self.is_connected, self.clientID, error

    def disconnect(self) -> None:
        '''
            Desconect cliente do coppelia caso esteja conectado.

        '''

        if (not self.is_connected):
            return
        
        self.log('Closing connection...')
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        # You can guarantee this with (for example):
        sim.simxGetPingTime(self.clientID)
        sim.simxFinish(self.clientID) # Now close the connection to CoppeliaSim:

        # self.stop()
        self.is_connected = False
        self.log('Connection closed')

    def start(self) -> None:
        '''
            Inicia simulacao caso haja conexao ativa.

        '''

        if (self.is_connected):
            sim.simxStartSimulation(self.clientID, sim.simx_opmode_blocking)
            self.is_running = True

    def stop(self) -> None:
        '''
            Encerra simulacao caso esteja rodando.

        '''

        if self.is_running:
            sim.simxStopSimulation(self.clientID, sim.simx_opmode_blocking)
            self.is_running = False

    def log(self, msg: str) -> None:
        print('[conn] ', msg)
