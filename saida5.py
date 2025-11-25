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
drive_base.straight(-15)
drive_base.reset()
drive_base.settings(turn_rate=90, straight_speed=500)
drive_base.straight(180)
drive_base.turn(-70)
drive_base.straight(470)
drive_base.turn(45)
drive_base.straight(20)
motorc.run_angle(600, 600)
drive_base.settings(straight_speed=50)
drive_base.straight(-158)
wait(10)
drive_base.straight(70)
motorc.run_angle(600, -525)
wait(10)
drive_base.settings(straight_speed=500)
drive_base.straight(-160)
drive_base.turn(15)
motorc.run_angle(600, 600)
drive_base.straight(40)
drive_base.turn(-20)
wait(30)
motorc.run_angle(600, -100)
#volta para a base
drive_base.settings(straight_speed=800)
drive_base.straight(-60)
drive_base.turn(-60)
drive_base.settings(straight_speed=1000)
motorc.run_angle(1000, -800)
drive_base.straight(-600)
