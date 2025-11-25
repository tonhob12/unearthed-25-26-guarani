from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# --- Inicializa motores ---
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
motorc = Motor(Port.C)
motord = Motor(Port.D, Direction.CLOCKWISE, [12, 36, 12, 20])

# --- Inicializa DriveBase ---
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=120)
drive_base.use_gyro(True)

drive_base.straight(-10)
drive_base.reset()
# --- Missão inicial da base ---
drive_base.settings(straight_speed=800,)
drive_base.straight(450)
drive_base.turn(7)
drive_base.straight(490)
drive_base.turn(80.5)
motord.run_angle(1000, -365)
motorc.run_angle(2000, -2000)
drive_base.settings(straight_speed=100)
drive_base.straight(130)
motord.run_angle(600, 205)
drive_base.straight(40)
motorc.run_angle(2000, 1400)
wait(10)
motorc.run_angle(2000, -1400)
drive_base.straight(-150)
motord.run_angle(1000, 320)
drive_base.settings(straight_speed=1000, turn_rate=120)
drive_base.turn(40)
drive_base.settings(straight_speed=1000)
drive_base.straight(385)
motorc.run_angle(2000, 1800)
drive_base.turn(48)
motord.run_angle(2000, -500)
drive_base.settings(straight_speed=1000)
drive_base.straight(-10)
motord.run_angle(1000, 350)
drive_base.turn(-85)
drive_base.straight(-300)
drive_base.turn(-80)
drive_base.straight(-745)