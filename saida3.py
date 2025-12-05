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

drive_base.settings(turn_rate=120)
drive_base.straight(145)
drive_base.turn(90)
drive_base.settings(straight_speed=400)
drive_base.straight(625)
wait(10)
drive_base.straight(-40)
motorc.run_angle(500, 450)
wait(20)

#checar/ajustar linhas abaixo!
drive_base.straight(-120)
wait(20)
motorc.run_angle(1000, -1040)
drive_base.turn(-90)
drive_base.settings(straight_speed=600, turn_rate=120)
drive_base.straight(165)
drive_base.turn(90)
drive_base.straight(368)
drive_base.turn(90)
drive_base.straight(175)
drive_base.turn(-15)
motord.run_angle(1000, 2500)
drive_base.turn(15)
drive_base.straight(-150)
drive_base.turn(-70)
drive_base.settings(straight_speed=1000, turn_rate=120)
drive_base.straight(900)



