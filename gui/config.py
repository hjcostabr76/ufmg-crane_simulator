
''' ==========================================================
-- CONSTANTES ------------------------------------------------
========================================================== '''

TIMEOUT = 5000 # ms
PORT_DEFAULT = 19997 # Porta padrao da simulacao no coppelia
VELOCITY_MAX_LEVEL = 10
SCALE = 10 # Fator de escala para dimensoes dos objetos da simulacao

# Objetos da simulacao

SIM_BASE = 'Base_PS' # Base do guindaste
SIM_ARM_JOINT = 'Arm_actuator' # Lanca

SIM_CRAB = 'Crab_PS' # Guicho: Posicao
SIM_CRAB_JOINT = 'Crab_actuator' # Guincho: Junta

SIM_HOIST = 'Guincho_PS' # Garra: Desce & sobe deslocando o ima
SIM_HOIST_JOINT_ANGULAR = 'Revolute_hoist' # Garra: Junta de movimento rotacional
SIM_HOIST_JOINT_VERTICAL = 'Hoist_actuator' # Garra: Junta de monvimento vertical

SIM_MAGNET = 'suctionPad' # Ima
SIM_MAGNET_SCRIPT = 'actuateMagnetRemote' # Ima: Funcao que atualiza seu estado

SIM_CAM1 = 'Camera_Panoramica' # Camera panoramica
SIM_CAM2 = 'Camera_Guincho' # Camera do guincho

SIM_SENS_FORCE_CABIN = 'Force_sensor' # Sensor de forca na cabine
SIM_SENS_FORCE_CRAB = 'Force_sensor0' # Sensor de forca no guincho
SIM_SENS_VISION = 'Vision_sensor' # Sensor de visao no guincho
SIM_SENS_PROX = 'Proximity_sensor' # Sensor de proximidade no guincho
