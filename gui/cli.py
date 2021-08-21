import sys
import time
from Simulation import Simulation
from Controller import Controller

if __name__ != "__main__":
    sys.exit()

def log(msg: str) -> None:    
    print('[cli]', msg)

def run_command(func, velocity: int) -> None:
    func(velocity)
    time.sleep(1)
    func(0)

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

        # Hoist: Vertical
        if command == 'w': # Up
            run_command(controller.set_hoist_vertical_vel, -4)
        elif command == 's': # Down
            run_command(controller.set_hoist_vertical_vel, 4)

        # Hoist: Angular
        elif command == 'v': # Right
            run_command(controller.set_hoist_angular_vel, 4)
        elif command == 'c': # Left
            run_command(controller.set_hoist_angular_vel, -4)

        # Arm
        elif (command in ['a', 'd']):

            if command == 'a': # Right
                run_command(controller.set_arm_vel, -10)
            else: # Left
                run_command(controller.set_arm_vel, 10)

            log('Arm angle: ' + str(controller.get_arm_angle()))

        # Crab
        elif command == 'r': # Front
            run_command(controller.set_crab_vel, 4)
        elif command == 'q': # Back
            run_command(controller.set_crab_vel, -4)

        # Magnet
        elif command == 'e':
            is_magnet_on = controller.toggle_magnet_state()
            log('magnet is ' + 'on' if is_magnet_on else 'off')

        else:
            log('Invalid command: ' + command)

        controller.get_arm_angle()
        # log('Arm angle: ' + )
            
except ConnectionError as conn_error:
    print('\nFailure as trying to start simulation: ', conn_error)
    raise conn_error

except ConnectionError as conn_error:
    print('\nLost connection to simulation: ', conn_error)
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
