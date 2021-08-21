
''' ==========================================================
-- CONSTANTES ------------------------------------------------
========================================================== '''

TIMEOUT = 5000 # ms
PORT_DEFAULT = 19997 # Porta padrao da simulacao no coppelia
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


''' ==========================================================
-- Funcoes utilitarias ---------------------------------------
========================================================== '''


'''
    TODO: 2021-08-21 - Checar se no fim vai ter alguma...
'''