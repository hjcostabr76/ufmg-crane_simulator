from config import VELOCITY_MAX
import sys
import time
from Simulation import Simulation
from Controller import Controller

if __name__ != "__main__":
    sys.exit()

VELOCITY_DEFAULT = 10

def log(msg: str) -> None:    
    print('[cli]', msg)

def run_command(func, velocity: int) -> None:
    func(velocity)
    time.sleep(1)
    func(0)

def get_angle_txt(angle: float) -> str:
    return "{:0.2f}".format(angle) + ' degres'

def get_position_txt(position: float) -> str:
    return '{:0.2f}'.format(position) + ' meters'

simulation: Simulation = None

try:

    # Captura IP customizado (se houver)
    ip = sys.argv[1] if (len(sys.argv) > 1) else None
    if (len(sys.argv) > 1):
        ip = sys.argv[1]

    # Inicia simulacao
    simulation = Simulation()
    is_connected, client_id, conn_error = simulation.connect(ip) if ip else simulation.connect()
    if (not is_connected):
        raise ConnectionError(conn_error)

    # Iniciar controlador
    simulation.start()
    controller = Controller(simulation)
    time.sleep(1)

    # Hora do show
    is_magnet_on = False

    while True:

        if (not simulation.is_simulation_running()):
            raise ConnectionAbortedError('Lost connection to simulation')

        command = input('Command: ')

        # Arm
        if (command in ['a', 'd']):

            if command == 'a': # Right
                run_command(controller.set_arm_vel, -VELOCITY_DEFAULT)
            else: # Left
                run_command(controller.set_arm_vel, VELOCITY_DEFAULT)

            log('Arm angle: ' + get_angle_txt(controller.get_arm_angle()))
            continue

        # Hoist: Vertical
        if (command in ['w', 's']):
            
            if command == 'w': # Up
                run_command(controller.set_hoist_vertical_vel, -VELOCITY_DEFAULT)
            else: # Down
                run_command(controller.set_hoist_vertical_vel, VELOCITY_DEFAULT)

            log('Hoist height: ' + get_position_txt(controller.get_hoist_height()))
            continue

        # Hoist: Angular
        if (command in ['v', 'c']):
            
            if command == 'v': # Right
                run_command(controller.set_hoist_angular_vel, VELOCITY_DEFAULT)
            else: # Left
                run_command(controller.set_hoist_angular_vel, -VELOCITY_DEFAULT)

            log('Hoist angle: ' + get_angle_txt(controller.get_hoist_angle()))
            continue

        # Crab
        if (command in ['r', 'q']):
        
            if command == 'r': # Front
                run_command(controller.set_crab_vel, VELOCITY_DEFAULT)
            else: # Back
                run_command(controller.set_crab_vel, -VELOCITY_DEFAULT)

            log('Crab position: ' + get_position_txt(controller.get_crab_position()))
            continue

        # Magnet
        if command == 'e':
            is_magnet_on = controller.toggle_magnet_state()
            log('magnet is ' + 'on' if is_magnet_on else 'off')
            continue

        log('Invalid command: ' + command)
            
except ConnectionError as conn_error:
    print('\nFailure as trying to start simulation: ', conn_error)
    raise conn_error

except ConnectionError as conn_error:
    print('\nLost connestr()ction to simulation: ', conn_error)
    raise conn_error

except KeyboardInterrupt:
    pass

except Exception as error:
    print('\nFalha inesperada: ', error)
    raise error

finally:
    if simulation:
        simulation.disconnect()
    print('\n-- THE END --\n')
