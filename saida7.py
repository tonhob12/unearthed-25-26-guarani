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
drive_base.straight(250)
drive_base.turn(-90)
drive_base.straight(799)
drive_base.turn(90)
drive_base.straight(500)
drive_base.turn(14)
wait(20)
drive_base.straight(250)
drive_base.turn(-14)
drive_base.straight(30)
drive_base.stop()
#solta M15 (1)
motorc.run_angle(1000, 600)

#sai da m15(1)
drive_base.straight(-250)
drive_base.turn(-90)
motorc.run_angle(1000, -600)
#vai pros artefatos
drive_base.straight(490)
drive_base.turn(-90)
#solta os artefatos
drive_base.straight(120)
motord.run_angle(1000, -1000)
#sai dos artefatos e vai para o ultimo m15 
drive_base.straight(-110)
drive_base.turn(90)
drive_base.straight(140)
#para com o ultimo m15 no chão e termina