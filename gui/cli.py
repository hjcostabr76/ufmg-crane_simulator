import sys
import time
from Simulation import Simulation
from Controller import Controller

if __name__ != "__main__":
    sys.exit()

def log(msg: str) -> None:    
    print('cli ', msg)

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

    controller = Controller(client_id)
    conn_manager.start()
    time.sleep(2)
    # Iniciar controlador

except ConnectionError as conn_error:
    print('\Falha ao iniciar simulacao: ', conn_error)
    raise conn_error

except Exception as error:
    print('\nFalha inesperada: ', error)
    raise error

finally:
    if (conn_manager):
        conn_manager.disconnect()
    print('\n-- THE END --\n')
