
''' ==========================================================
-- CONSTANTES ------------------------------------------------
========================================================== '''

TIMEOUT = 5000 # ms
PORT_DEFAULT = 19997 # Porta padrao da simulacao no coppelia

# Objetos da simulacao
SIM_ARM = 'Arm_actuator' # Lanca
SIM_CRAB = 'Crab_actuator' # Guincho
SIM_HOIST = 'Hoist_actuator' # Garra
SIM_MAGNET = 'suctionPad' # Ima

SIM_CAM1 = 'Camera_Panoramica' # Camera panoramica
SIM_CAM2 = 'Camera_Guincho' # Camera do guincho

SIM_SENS_FORCE_CABIN = 'Force_sensor' # Sensor de forca na cabine
SIM_SENS_FORCE_CRAB = 'Force_sensor0' # Sensor de forca no guincho
SIM_SENS_VISION = 'Vision_sensor' # Sensor de visao no guincho
SIM_SENS_PROX = 'Proximity_sensor' # Sensor de proximidade no guincho


''' ==========================================================
-- Funcoes utilitarias ---------------------------------------
========================================================== '''


def handle_coppelia_return_code(return_code: int):

    '''
        TODO: 2021-08-20 - Implementar
        See: https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
    '''

    # sim.simx_return_ok
    # sim.simx_return_novalue_flag
    # sim.simx_return_timeout_flag
    # sim.simx_return_illegal_opmode_flag
    # sim.simx_return_remote_error_flag
    # sim.simx_return_split_progress_flag
    # sim.simx_return_local_error_flag
    # sim.simx_return_initialize_error_flag
    
    raise NotImplementedError()