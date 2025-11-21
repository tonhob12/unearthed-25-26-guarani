from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# --- Inicializa motores ---
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
motorc = Motor(Port.C)
motord = Motor(Port.D)

# --- Inicializa DriveBase ---
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=120)
drive_base.use_gyro(True)

# --- Missão inicial da base ---
drive_base.straight(-10)
drive_base.reset()
drive_base.settings(straight_speed=800)
drive_base.straight(150)
drive_base.turn(10)
drive_base.straight(507)
drive_base.turn(-96)
drive_base.settings(straight_speed=200)
drive_base.straight(150)

# --- TRAVA a base para executar o motor D ---
drive_base.stop()        # trava as rodas da base
motord.run_angle(1000, -2000)     # executa o motor D enquanto a base está parada

# --- Após missão do motor D, libera a base ---

# --- Continua a missão ---
drive_base.turn(20)
drive_base.turn(-40)
drive_base.turn(25)

drive_base.straight(40)

motord.run_angle(1000, 2000)
drive_base.straight(65)
# --- Braço secundário ---
motorc.run_angle(1000, -1350)
wait(10)
drive_base.settings(straight_speed=100)
drive_base.straight(-40)

drive_base.straight(60)

drive_base.straight(-190)

motorc.run_angle(1000, 860)

drive_base.turn(-75)
drive_base.settings(straight_speed=1000)
drive_base.straight(780)