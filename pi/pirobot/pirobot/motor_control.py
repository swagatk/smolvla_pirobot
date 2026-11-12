# motor control code


# load the libraries
import json
import os

import RPi.GPIO as GPIO


# set variables for GPIO  Pins
pinMotorAForward = 9
pinMotorABackward = 10
pinMotorBForward = 8
pinMotorBBackward = 7


# PWM parameters. DutyCycleA/B (and Frequency) are overridden by the
# calibration file written by `ros2 run pirobot calibrate_motors`.
# Motor A is the left wheel, motor B the right (see turnleft()).
Frequency = 20
DutyCycleA = 50
DutyCycleB = 50
Stop = 0

CALIB_FILE = os.environ.get(
    'PIROBOT_MOTOR_CALIB',
    os.path.expanduser('~/.config/pirobot/motor_calib.json'))

# Commands that move the robot; anything else stops it.
ACTIVE_COMMANDS = ('UP', 'DOWN', 'LEFT', 'RIGHT')

_initialized = False


def load_calibration(path=CALIB_FILE):
    """Load duty_a, duty_b, frequency from the calibration file, if present.
    Returns the path loaded, or None (defaults kept)."""
    global DutyCycleA, DutyCycleB, Frequency
    try:
        with open(path) as f:
            c = json.load(f)
    except FileNotFoundError:
        return None
    DutyCycleA = float(c.get('duty_a', DutyCycleA))
    DutyCycleB = float(c.get('duty_b', DutyCycleB))
    Frequency = float(c.get('frequency', Frequency))
    return path


def save_calibration(path=CALIB_FILE):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        json.dump({'duty_a': DutyCycleA, 'duty_b': DutyCycleB,
                   'frequency': Frequency}, f, indent=2)
    return path


def set_duty(a, b):
    """Set the duty cycles used by forward()/backward()/turns (0..100).
    Takes effect on the next motion call."""
    global DutyCycleA, DutyCycleB
    DutyCycleA = max(0.0, min(100.0, float(a)))
    DutyCycleB = max(0.0, min(100.0, float(b)))


def initialize_motors(calibrate=True):
    """Set up GPIO/PWM. With calibrate=True, load the calibration file first
    (it may change Frequency, which is fixed once PWM starts)."""
    global _initialized
    if calibrate:
        load_calibration()
    #set GPIO modes
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Set the GPIO Pin Mode
    GPIO.setup(pinMotorAForward, GPIO.OUT)
    GPIO.setup(pinMotorABackward, GPIO.OUT)
    GPIO.setup(pinMotorBForward, GPIO.OUT)
    GPIO.setup(pinMotorBBackward, GPIO.OUT)

    global pwmMotorAForward,\
       pwmMotorBForward,\
       pwmMotorABackward,\
       pwmMotorBBackward

    # Set the GPIO to Software PWM at 'Frequency' Hertz
    pwmMotorAForward = GPIO.PWM(pinMotorAForward, Frequency)
    pwmMotorABackward = GPIO.PWM(pinMotorABackward, Frequency)
    pwmMotorBForward = GPIO.PWM(pinMotorBForward, Frequency)
    pwmMotorBBackward = GPIO.PWM(pinMotorBBackward, Frequency)

    # set the duty cycle for software PWM - initially to 0
    pwmMotorAForward.start(Stop)
    pwmMotorABackward.start(Stop)
    pwmMotorBForward.start(Stop)
    pwmMotorBBackward.start(Stop)
    _initialized = True


def cleanup():
    """Stop the motors and release the GPIO pins (safe to call twice)."""
    global _initialized
    if not _initialized:
        return
    stopmotors()
    for pwm in (pwmMotorAForward, pwmMotorABackward,
                pwmMotorBForward, pwmMotorBBackward):
        pwm.stop()
    GPIO.cleanup((pinMotorAForward, pinMotorABackward,
                  pinMotorBForward, pinMotorBBackward))
    _initialized = False

def stopmotors():
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def forward():
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)

def backward():
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)

def turnleft():
    pwmMotorAForward.ChangeDutyCycle(Stop)
    pwmMotorABackward.ChangeDutyCycle(DutyCycleA)
    pwmMotorBForward.ChangeDutyCycle(DutyCycleB)
    pwmMotorBBackward.ChangeDutyCycle(Stop)


def turnright():
    pwmMotorAForward.ChangeDutyCycle(DutyCycleA)
    pwmMotorABackward.ChangeDutyCycle(Stop)
    pwmMotorBForward.ChangeDutyCycle(Stop)
    pwmMotorBBackward.ChangeDutyCycle(DutyCycleB)


def key_control(kp):

    if kp.getKey('LEFT'):
        turnleft()

    elif kp.getKey('RIGHT'):
        turnright()

    elif kp.getKey('UP'):
        forward()

    elif kp.getKey('DOWN'):
        backward()

    elif kp.getKey('ESCAPE'):
        return False

    else:
        stopmotors()
    return True


def exec_cmd(cmd_str):
    """Apply a /pirobot/command string. Unknown commands, 'STOP' and 'EXIT' stop."""
    if cmd_str == 'LEFT':
        turnleft()
    elif cmd_str == 'RIGHT':
        turnright()
    elif cmd_str == 'UP':
        forward()
    elif cmd_str == 'DOWN':
        backward()
    else:
        stopmotors()


if __name__ == '__main__':
    # Local test without ROS: drive with the arrow keys in the pygame window.
    import time
    from pirobot import key_press as kp
    initialize_motors()
    kp.init()
    print('Controlling using keyboard, ESC to quit')
    try:
        while key_control(kp):
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
        kp.stop()
