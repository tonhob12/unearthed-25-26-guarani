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
drive_base.straight(200)
drive_base.turn(2)
drive_base.straight(210)
drive_base.stop()
for i in range(3):
    motorc.run_angle(800, 460)
    wait(10)
    motorc.run_angle(300, -500)

drive_base.settings(straight_speed=1000)
drive_base.straight(-410)