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
drive_base.straight(-15)
drive_base.reset()
drive_base.settings(turn_rate=120, straight_speed=500)
drive_base.straight(180)
drive_base.turn(-70)
drive_base.straight(470)
drive_base.turn(46)
drive_base.straight(25)
motorc.run_angle(1000, 700)
drive_base.settings(straight_speed=100)
drive_base.straight(-156)
wait(10)
drive_base.straight(50)
motorc.run_angle(1000, -525)
wait(10)
drive_base.settings(straight_speed=500)
drive_base.straight(-148)
drive_base.turn(15)
motorc.run_angle(1000, 560)
drive_base.straight(60)
drive_base.turn(-20)
wait(30)
motorc.run_angle(1000, -100)
#volta para a base
drive_base.settings(straight_speed=800)
drive_base.straight(-60)
drive_base.turn(-60)
drive_base.settings(straight_speed=1000)
async def andarwhilelevantamotor():
    await multitask(drive_base.straight(-600), motorc.run_angle(1000, -800))
run_task(andarwhilelevantamotor())


