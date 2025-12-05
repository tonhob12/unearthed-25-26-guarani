from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, multitask, run_task


# --- Inicializa motores ---
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
motorc = Motor(Port.C)
motord = Motor(Port.D)

# --- Inicializa DriveBase ---
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=120)
drive_base.use_gyro(True)

drive_base.settings(turn_rate=120, straight_speed=500)
drive_base.straight(-15)
drive_base.reset()
drive_base.straight(100)
drive_base.turn(-15)
drive_base.straight(465)
drive_base.turn(63)
drive_base.straight(260)
drive_base.stop()
motord.run_angle(1000, 1450)
wait(20)
drive_base.settings(straight_speed=1000, turn_rate=90)
drive_base.straight(-280)
drive_base.turn(-20)
motord.run_angle(1000, -524)
drive_base.straight(88)
drive_base.settings(turn_rate=50)
drive_base.turn(-33)
drive_base.straight(-100)
drive_base.straight(-280)
drive_base.settings(straight_speed=1000, turn_rate=120)
drive_base.turn(-45)
async def andarwhilelevantamotor():
    await multitask(drive_base.straight(-600), motord.run_angle(1000, 525))
run_task(andarwhilelevantamotor())

