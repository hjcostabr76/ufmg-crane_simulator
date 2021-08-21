import sys
import time
from Simulation import Simulation
from Controller import Controller

if __name__ != "__main__":
    sys.exit()

def log(msg: str) -> None:    
    print('cli ', msg)

def run_command(func, velocity: int) -> None:
    func(velocity)
    time.sleep(1)
    func(0)

conn_manager: Simulation = None

try:

    # Captura IP customizado (se houver)
    ip = sys.argv[1] if (len(sys.argv) > 1) else None
    if (len(sys.argv) > 1):
        ip = sys.argv[1]

    # Inicia simulacao
    conn_manager = Simulation()
    is_connected, client_id, conn_error = conn_manager.connect(ip) if ip else conn_manager.connect()
    if (not is_connected):
        raise ConnectionError(conn_error)

    # Iniciar controlador
    conn_manager.start()
    controller = Controller(conn_manager)
    time.sleep(1)

    # Hora do show
    is_magnet_on = False

    while True:

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
        elif command == 'a': # Right
            run_command(controller.set_arm_vel, -4)
        elif command == 'd': # Left
            run_command(controller.set_arm_vel, 4)

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
            
except ConnectionError as conn_error:
    print('\nFalha ao iniciar simulacao: ', conn_error)
    raise conn_error

except KeyboardInterrupt:
    pass

except Exception as error:
    print('\nFalha inesperada: ', error)
    raise error

finally:
    if (conn_manager):
        conn_manager.disconnect()
    print('\n-- THE END --\n')
