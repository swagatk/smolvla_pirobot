# motor control code


# load the libraries
import RPi.GPIO as GPIO


# set variables for GPIO  Pins
pinMotorAForward = 9
pinMotorABackward = 10
pinMotorBForward = 8
pinMotorBBackward = 7


# PWM parameters
Frequency = 20
DutyCycleA = 50
DutyCycleB = 50
Stop = 0

# Commands that move the robot; anything else stops it.
ACTIVE_COMMANDS = ('UP', 'DOWN', 'LEFT', 'RIGHT')

_initialized = False


def initialize_motors():
    global _initialized
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
