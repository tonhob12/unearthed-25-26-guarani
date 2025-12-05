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
drive_base.settings(straight_speed=600)
drive_base.straight(200)
drive_base.turn(1)
motorc.run_angle(1000, 150)
drive_base.straight(210)
drive_base.stop()
for i in range(3):
    motorc.run_angle(850, 400)
    wait(10)
    motorc.run_angle(1000, -400)

drive_base.settings(straight_speed=1000)
drive_base.straight(-410)