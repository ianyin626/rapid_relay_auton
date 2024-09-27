# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       Library                                                      #
# 	Author:       Ian Yin                                                      #
# 	Created:      8/14/2023, 9:58:04 PM                                        #
# 	Description:  The Library with all the functions                           #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
# Brain should be defined by default
brain = Brain()
brain_inertial = Inertial()
left_motor = Motor(Ports.PORT1, False)
right_motor = Motor(Ports.PORT7, True)
motor_group_left_1 = Motor(Ports.PORT2, False)
motor_group_left_2 = Motor(Ports.PORT3, True)
motor_group_right_1 = Motor(Ports.PORT8, True)
motor_group_right_2 = Motor(Ports.PORT9, False)
all_lift_motors = MotorGroup(motor_group_left_1, motor_group_left_2, motor_group_right_1, motor_group_right_2)
touchled = Touchled(Ports.PORT5)
pneumatic = Pneumatic(Ports.PORT4, False)
controller = Controller()

# Settings
all_lift_motors.set_stopping(BrakeType.BRAKE)
all_lift_motors.set_velocity(100, PERCENT)
all_lift_motors.set_max_torque(100, PERCENT)

# Variables
MOTOR_RATIO = 1
PI = 3.141592653589893
GEAR_RATIO = 3/2
DEBUG = False
LIFT_DEGREES_DUMP_PURPLE = -783
LIFT_DEGREES_DUMP_TOP_GREEN = -245
LIFT_DEGREES_HOLD_PURPLE = -630
LIFT_DEGREES_HOLD_GREEN = -530

# Global variables
intake_is_spinning = False
cylinder_extended = {
    str(CYLINDER1): False,
    str(CYLINDER2): False
}

def init():
    # set motor velocity
    all_lift_motors.set_max_torque(100, PERCENT)
    all_lift_motors.set_velocity(100, PERCENT)

    pneumatic.retract(CYLINDERALL)

# Pre-defined functions
def move(left_speed, right_speed):
    # log("DEBUG", 'left_speed: {0:.2f}, right_speed: {1:.2f}'.format(left_speed, right_speed))
    if (MOTOR_RATIO > 1):
        left_motor.set_velocity(left_speed, PERCENT)
        right_motor.set_velocity(right_speed / MOTOR_RATIO, PERCENT)
    else:
        left_motor.set_velocity(left_speed * MOTOR_RATIO, PERCENT)
        right_motor.set_velocity(right_speed, PERCENT)
    left_motor.spin(FORWARD)
    right_motor.spin(FORWARD)

def stop_driving():
    left_motor.stop()
    right_motor.stop()

def detectMotorStall(motor:Motor|MotorGroup, maxCurrent:float = 0.5, minVelocity:int = 10):
    if (motor.current(CurrentUnits.AMP) > maxCurrent and abs(motor.velocity(PERCENT)) < minVelocity):
        return True
    else:
        return False

def error_monitor(error, red_error, yellow_error):
    if (abs(error) > red_error):
        touchled.set_color(Color.RED)
    elif(abs(error) > yellow_error):
        touchled.set_color(Color.ORANGE)
    else:
        touchled.set_color(Color.GREEN)

def short_wait():
    wait(0.1, SECONDS)
    
# Normal functions
def PID_turn(target_rotation:int, kp:float, kd, ki:float, max_error:int):
    # Make sure to set the motor stopping
    minSpeed = 7
    direction = 1
    speed = minSpeed
    if (target_rotation < brain_inertial.rotation()):
        speed = minSpeed * -1
        direction = -1

    error_total = brain_inertial.rotation() - target_rotation
    last_error = error_total
    # brain.timer.reset()
    while (abs(target_rotation - brain_inertial.rotation()) > max_error):
        error = target_rotation - brain_inertial.rotation()
        derivative = error - last_error
        integral = ki * error_total
        if (abs(error * kp) > minSpeed):
            speed = (error * kp) + (derivative * kd) + (ki * integral)
        if (abs(speed) < minSpeed):
            speed = minSpeed * direction
        # if (detectMotorStall(left_motor, 0.5, 2)):
        #     brain.play_sound(SoundType.SIREN)
        #     speed = speed * 2
        move(speed, speed * -1)
        error_total = error_total + error
        wait(10, MSEC)
        last_error = error
    stop_driving()
    wait(20, MSEC)
    if (abs(target_rotation - brain_inertial.rotation()) > max_error * 2):
        brain.play_sound(SoundType.TADA)
        PID_turn(target_rotation, kp, kd, ki, max_error)

def drive_straight_PID(target_distance:int, speed:int, kp:float, kd:float, ki:float, target_heading:int):
    left_motor.set_stopping(BRAKE)
    right_motor.set_stopping(BRAKE)
    left_motor.set_position(0, TURNS)
    right_motor.set_position(0, TURNS)
    global error_total
    error_total = brain_inertial.rotation() - target_heading
    while (((abs(left_motor.position(TURNS)) + abs(right_motor.position(TURNS))) / 2) < (target_distance / (GEAR_RATIO*200))):
        error = brain_inertial.rotation() - target_heading
        last_error = error
        wait(20, MSEC)
        error = brain_inertial.rotation() - target_heading
        derivative = error - last_error
        output = (error * kp) + (derivative * kd) + (0.5 * (error_total * ki))
        move(speed - output, speed + output)
        error_total = error_total + error
    stop_driving()

def move_to_stall(speed:int):
    move(speed, speed)
    wait(0.5, SECONDS)
    while (detectMotorStall(right_motor) == False):
        wait(50, MSEC)
    short_wait()
    stop_driving()

def dump_purple():
    cylinder_action(CYLINDER1)
    cylinder_action(CYLINDER2)

def cylinder_action(cylinder):
    global cylinder_extended
    if cylinder_extended[str(cylinder)]:
        pneumatic.retract(cylinder)
    else:
        pneumatic.extend(cylinder)

    cylinder_extended[str(cylinder)] = not cylinder_extended[str(cylinder)]

def steering_drive(speed:int, steering:int):
    if (steering > 0):
        drive_right_velocity = speed - (speed * (steering / 50))
        drive_left_velocity = speed
    else:
        drive_left_velocity = speed + (speed * (steering / 50))
        drive_right_velocity = speed
    move(drive_left_velocity, drive_right_velocity)

def turn_to_rotation(target_rotation, momentum, stop_when_stall=False, speed:int=35):
    left_motor.set_stopping(BRAKE)
    right_motor.set_stopping(BRAKE)
    stop_driving()
    wait(100, MSEC)
    left_motor.set_position(0, TURNS)
    right_motor.set_position(0, TURNS)
    error = target_rotation - brain_inertial.rotation(DEGREES)
    # speed = 30
    distance_error = (PI * (220 * (abs(error) - momentum) / 360))
    while (((abs(left_motor.position(TURNS)) + abs(right_motor.position(TURNS))) / 2) < (distance_error / (GEAR_RATIO * 200))):
        if (error > 0):
           move(speed, speed * -1)
        else:
           move(speed * -1, speed)
        wait(10, MSEC)
        # Exit if robot stuck
        if (stop_when_stall and detectMotorStall(left_motor, 0.8, 1)):
            stop_driving()
            return False # Stuck!
    stop_driving()
    return True

def move_to_stall_straight(Left_motor_speed, Right_motor_speed, heading, kp):
    move(Left_motor_speed, Right_motor_speed)
    wait(0.5, SECONDS)
    while (detectMotorStall(left_motor, 0.7, 10) == False):
        error = brain_inertial.rotation(DEGREES) - heading
        output = error * kp
        move(Left_motor_speed - output, Right_motor_speed + output)
        wait(20, MSEC)

def log(level, message):
    print('{0:.2f} {1} {2}'.format(brain.timer.time(SECONDS), level, message))

def datalogger():
    if brain.sdcard.is_inserted():
        log("INFO", 'SD card is inserted')
        brain.sdcard.appendfile('data.csv', bytearray("Time,left_motorRPM\n")) # type: ignore
    while True:
        data = "{0:.2f},{1:.2f}\n".format(brain.timer.time(SECONDS), left_motor.velocity(RPM))
        brain.sdcard.appendfile('data.csv', bytearray(data)) # type: ignore
        wait(50, MSEC)
if DEBUG:
    dataloggerThread = Thread(datalogger)

def moveLiftToBottom():
    all_lift_motors.spin(FORWARD)
    while detectMotorStall(all_lift_motors, 0.8, 2) == False:
        wait(10, MSEC)
    all_lift_motors.stop()

def debug_step():
    if (DEBUG == False):
        return

    touchled.set_blink(0.25, 0.25)
    while (not(touchled.pressing())):
        wait(20, MSEC)
    touchled.on()


# The program

def run_route_old():
    drive_straight_PID(550, 100, 1, 0.004, 0.003, 0)
    turn_to_rotation(90, 3)
    debug_step()    



def the_program():
    init()
    brain.timer.reset()
    run_route_old()
    brain.screen.print(brain.timer.time(SECONDS))
    log("INFO", 'timer: {0:.2f}'.format(brain.timer.time((SECONDS))))

the_program()