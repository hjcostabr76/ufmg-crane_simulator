from common import *

try:
    import sim
except:
    print ('Error import sim. Verify files and settings Coppelia.')

class Controller:

    '''
        TODO: 2021-08-20 - ADD Descricao
    '''

    client_id: int = None

    arm = None
    crab = None
    hoist = None
    magnet = None
    
    cam1 = None
    cam2 = None
    
    sensor_force_cabin = None
    sensor_force_crab = None
    sensor_vision = None
    sensor_prox = None


    def __init__(self, client_id: int) -> None:

        self.client_id = client_id

        # Captura handlers dos objetos da simulacao
        self.arm = self.get_object_handle(SIM_ARM)
        self.crab = self.get_object_handle(SIM_CRAB)
        self.hoist = self.get_object_handle(SIM_HOIST)
        self.magnet = self.get_object_handle(SIM_MAGNET)
        self.cam1 = self.get_object_handle(SIM_CAM1)
        self.cam2 = self.get_object_handle(SIM_CAM2)
        self.sensor_force_cabin = self.get_object_handle(SIM_SENS_FORCE_CABIN)
        self.sensor_force_crab = self.get_object_handle(SIM_SENS_FORCE_CRAB)
        self.sensor_vision = self.get_object_handle(SIM_SENS_VISION)
        self.sensor_prox = self.get_object_handle(SIM_SENS_PROX)

    def get_object_handle(self, obj_name: str):
        '''
            TODO: 2021-08-20 - Tipar o retorno
        '''
        handle_result = sim.simxGetObjectHandle(self.client_id, obj_name, sim.simx_opmode_blocking)
        validate_coppelia_return(handle_result[0])
        self.log('Handling object: ' + obj_name)
        return handle_result[1]

    def log(self, msg: str) -> None:
        print('[controller]', msg)
