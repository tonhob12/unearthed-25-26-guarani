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

#alinhamento automático
drive_base.straight(-15)
drive_base.reset()

#sai da base
drive_base.straight(320)
drive_base.turn(-90)
drive_base.settings(straight_speed=400)
drive_base.straight(1350)
drive_base.turn(90)
drive_base.straight(70)
drive_base.turn(90)
motord.run_angle(1000, -500)
drive_base.straight(-150)
drive_base.turn(-85)
drive_base.straight(200)
drive_base.turn(22)
drive_base.straight(106)
