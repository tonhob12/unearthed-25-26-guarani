from pybricks.hubs import PrimeHub
from pybricks.parameters import Button, Port, Direction
from pybricks.tools import wait

# --- Classe de motor falso (simulação) ---
class FakeMotor:
    def __init__(self, port, direction=None):
        self.port = port
        self.direction = direction
        print(f"[Simulando] Motor na porta {port} (direção={direction})")

    def run(self, speed):
        print(f"[{self.port}] Simulado rodando a {speed} deg/s")

    def stop(self):
        print(f"[{self.port}] Simulado parado")

# --- Função que tenta criar o motor real; se falhar, usa o simulado ---
def criar_motor(porta, direction=Direction.CLOCKWISE):
    try:
        from pybricks.pupdevices import Motor
        return Motor(porta, positive_direction=direction)
    except Exception as e:
        print(f"  Porta {porta} sem motor real ({e}). Usando simulação.")
        return FakeMotor(porta, direction)

# --- Inicialização ---
hub = PrimeHub()
motora = criar_motor(Port.A)
motorb = criar_motor(Port.B, Direction.COUNTERCLOCKWISE)
motorc = criar_motor(Port.C)
motord = criar_motor(Port.D)

# --- Loop principal ---
while True:
    botoes = hub.buttons.pressed()

    if Button.RIGHT in botoes:
        motora.run(500)
        motorb.run(500)
        motorc.run(500)
        motord.run(500)

    elif Button.LEFT in botoes:
        motora.run(-500)
        motorb.run(-500)
        motorc.run(-500)
        motord.run(-500)

    else:
        motora.stop()
        motorb.stop()
        motorc.stop()
        motord.stop()

    wait(50)