
'''
    NOTE: Valores para posicionar o guindaste sobre os containers (braco, crab, altura do guinco):
    - Posicao para container 1.1: 36 69 25
    - Posicao para container 1.2: ??
    - Posicao para container 2.1: ??
    - Posicao para container 2.2: ??
'''

'''
    TODO: 2021-08-27 - Checar valor de leitura da altura do guinco
'''

'''
    TODO: 2021-08-27 - Altear os seguintes parametros da simulacao:

    - O que a gente usa eh o sensor de visao e nao a camera;
    - Se o objetivo for fazer aparecer na gui as mesmas imagens que estao aparecendo nas cameras da simulacao, vamos precisar:
        - Colocar o sensor de visao 'Vision_sensor' no lugar aonde esta o elemento 'Camera_Guincho';
        - Colocar 01 novo sensor de visao no local aonde esta o elemento 'Camera_Panoramica';
    - Alterar configuracoes do 'Vision_sensor':
        - Marcar checkbox 'Perspective Mode';
        - Alterar resolucao para 256 x 256;
    - O sensor de proximidade ta pegando a distancia para um elemento aleatorio que eu ainda nao descobri qual eh (handle: 98).
        - Alguem sabe se da pra descobrir o elemento da simulacao pelo codigo (handle)?
'''


''' ==========================================================
-- CONSTANTES ------------------------------------------------
========================================================== '''

NAME_SIMULATION = 'Crane Simulator - UFMG: Systems Engineering - Group B - 2021/1'

TIMEOUT_CONNECTION = 5000 # ms
TIMEOUT_DISPLAY = 300 # Intervalo de atualizacao dos valores de telemetria em ms 

SCALE = 10 # Fator de escala para dimensoes dos objetos da simulacao
PORT_DEFAULT = 19997 # Porta padrao da simulacao no coppelia
LCD_DIGITS_COUNT = 7 # Maximo de digitos a serem exibidos nos displays lcd
VELOCITY_MAX_LEVEL = 10

# Objetos da simulacao

SIM_BASE = 'Base_PS' # Base do guindaste
SIM_ARM_JOINT = 'Arm_actuator' # Lanca

SIM_CRAB = 'Crab_PS' # Guicho: Posicao
SIM_CRAB_JOINT = 'Crab_actuator' # Guincho: Junta

SIM_HOIST = 'Guincho_PS' # Garra: Desce & sobe deslocando o ima
SIM_HOIST_JOINT_ANGULAR = 'Revolute_hoist' # Garra: Junta de movimento rotacional
SIM_HOIST_JOINT_VERTICAL = 'Hoist_actuator' # Garra: Junta de monvimento vertical

SIM_MAGNET = 'suctionPad' # Ima
SIM_MAGNET_SCRIPT = 'actuateMagnet' # Ima: Funcao que atualiza seu estado

SIM_SENS_VISION_CAM1 = 'Visao_panoramica' # Camera panoramica
SIM_SENS_VISION_CAM2 = 'Visao_guincho' # Camera do guincho
SIM_SENS_FORCE = 'Force_sensor0' # Sensor de forca no guincho
SIM_SENS_PROX = 'Proximity_sensor' # Sensor de proximidade no guincho
