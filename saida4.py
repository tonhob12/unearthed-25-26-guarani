from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.tools import wait, StopWatch, multitask, run_task
from pybricks.robotics import DriveBase
from umath import pi

global hub, motorc, right_motor, left_motor, motord, drive_base,Chassis_sensor,color_sensor
hub = PrimeHub()
hub.system.set_stop_button(Button.CENTER)
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)
motorc = Motor(Port.C)
motord = Motor(Port.D)

# --- Inicializa DriveBase ---
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=62.4, axle_track=125)
drive_base.use_gyro(True)
drive_base.settings(turn_rate=60 * 3.6)
timer_move = StopWatch()

def Gyro():
    Gyro = hub.imu.heading()
    Gyro = Gyro % 360 #0 a 360 e nao de 0 a 180 ou 0 a -180
    if  Gyro > 180:
        Gyro -= 360
    return Gyro

def pd_move_loop(Power, Kp, Kd, prev_err):
    angle_B = right_motor.angle() #360
    angle_A = left_motor.angle() #350
    err = angle_B - angle_A

    P = Kp * err
    D = Kd * (err - prev_err)
    pid_adj = P + D

    right_motor.dc(Power - pid_adj)
    left_motor.dc(Power + pid_adj)

    wait(10)
    return err


def pd_move_enc(Power, Kp, Kd, Deg, Brake=True):
    prev_err = 0
    right_motor.reset_angle(0)
    left_motor.reset_angle(0)

    while (abs(right_motor.angle()) + abs(left_motor.angle())) / 2 <= Deg:
        prev_err = pd_move_loop(Power, Kp, Kd, prev_err)

    if Brake:
        drive_base.stop()


def pd_move_spin(Power, Kp, Kd, Total_Deg, Brake=True):
    right_motor.reset_angle(0)
    left_motor.reset_angle(0)
    prev_err = 0

    while (abs(right_motor.angle()) + abs(left_motor.angle())) / 2 <= Total_Deg:
        angle_B = right_motor.angle()
        angle_C = left_motor.angle()

        
        err = angle_B + angle_C

        P = Kp * err
        D = Kd * (err - prev_err)
        pid_adj = P + D

        right_motor.dc(Power - pid_adj)   
        left_motor.dc(-Power - pid_adj)  

        prev_err = err
        wait(10)

    if Brake:
        drive_base.stop()


def move_time(Power_B, Power_C, ms, Brake=True):
    timer_move.reset()
    while timer_move.time() < ms:
        right_motor.dc(Power_B)
        left_motor.dc(Power_C)
    if Brake:
        drive_base.stop()


def move_deg(Power_B, Power_C, Deg, Brake=True):
    right_motor.reset_angle(0)
    left_motor.reset_angle(0)
    while (abs(right_motor.angle()) + abs(left_motor.angle())) / 2 <= Deg:
        right_motor.dc(Power_B)
        left_motor.dc(Power_C)
    if Brake:
        drive_base.stop()


def turn_gyro(hdg, Brake=True):
    drive_base.settings(turn_rate=60 * 3.6)
    drive_base.turn(hdg)
    if Brake:
        drive_base.stop()

def turn_gyrospeed(speed,hdg, Brake=True):
    drive_base.settings(turn_rate= speed * 3.6)
    drive_base.turn(hdg)
    if Brake:
        drive_base.stop()

def gyro_track(
    Kp, Kd,
    Deg1, Deg2,
    GyroDeg,
    Lowpower, Bigpower, Lowpower2,
    Deg,
    Brake=True
):
    right_motor.reset_angle(0)
    left_motor.reset_angle(0)

    direction = 1 if Deg > 0 else -1
    target = abs(Deg)

    last_error = 0

    while True:
        motor_deg = abs((right_motor.angle() + left_motor.angle()) / 2)

        if motor_deg >= target:
            break

        # -------- PERFIL DE VELOCIDADE (SEMPRE POSITIVO) --------
        if motor_deg < Deg1:
            power = map_value(
                motor_deg,
                0, Deg1,
                Lowpower, Bigpower
            )
        elif motor_deg < target - Deg2:
            power = Bigpower
        else:
            power = map_value(
                motor_deg,
                target - Deg2, target,
                Bigpower, Lowpower2
            )

        # -------- PD DO GYRO --------
        error = GyroDeg - Gyro()
        derivative = error - last_error
        correction = (Kp * error) + (Kd * derivative)

        right_motor.dc(direction * power - correction)
        left_motor.dc(direction * power + correction)

        last_error = error
        wait(10)

    if Brake:
        bc_stop()


def gyro_turn_pd(TargetDeg, Kp, Kd, MaxPower, MinPower, Brake=True):
    start_angle = Gyro()
    last_error = 0
    stable = 0

    while True:
        current = Gyro() - start_angle
        error = TargetDeg - current

        # normaliza
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        if abs(error) < 1.0:
            stable += 1
            if stable > 6:
                break
        else:
            stable = 0

        derivative = error - last_error
        correction = Kp * error + Kd * derivative

        # clamp
        correction = max(-MaxPower, min(MaxPower, correction))

        # torque mínimo
        if abs(correction) < MinPower:
            correction = MinPower if correction > 0 else -MinPower

        right_motor.dc(-correction)
        left_motor.dc(correction)

        last_error = error
        wait(10)

    if Brake:
        bc_stop()

def c_motor(Power, Deg, is_wait=True):
    motorc.run_angle(Power * 11, Deg, wait=is_wait)

def d_motor(Power, Deg, is_wait=True):
    motord.run_angle(Power * 11, Deg, wait=is_wait)

def bc_stop():
    right_motor.dc(0)
    left_motor.dc(0)

def map_value(value, from_low, from_high, to_low, to_high):
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low

move_time(-60, -60, 200)
drive_base.reset()
hub.imu.reset_heading(0)
motorc.run_time(-1000, 500)
motorc.reset_angle(0)
gyro_track(2.2,4.2,150,150,0, 40, 80, 30, 300)
gyro_turn_pd(-28, 1.2, 5.2, 80, 32)
gyro_track(2.2, 4.2, 300, 300, -28, 40, 80, 30, 730)
gyro_turn_pd(28, 1.2, 5.2, 80, 32)
gyro_track(2.2, 4.2, 25, 25, 0, 25, 50, 25, 50)
c_motor(100, 115)
wait(300)
for i in range(3):
    c_motor(100, 285)
    c_motor(100, -285)
    wait(100)
c_motor(100, -115)
gyro_turn_pd(58, 1.2, 5.2, 80, 32)
d_motor(100, 220)
move_time(80, 80, 2100)
d_motor(100,1460)
gyro_track(2.2, 4.2, 300, 300, 58, 40, 80, 30, -250)
gyro_turn_pd(-58, 1.2, 5.2, 80, 32)
gyro_track(2.2, 4.2, 300, 300, 0, 40, 80, 30, -800)
gyro_track(2.2, 4.2, 300, 300, -70, 40, 80, 30, -1500)