"""Interactive motor trim: find DutyCycleA/B so 'UP' drives straight.

Runs on the Raspberry Pi in a plain terminal (works over SSH). Stop
motion_subscriber / robot.launch.py first: both use the same GPIO pins.

    ros2 run pirobot calibrate_motors
    ros2 run pirobot calibrate_motors --duration 4 --frequency 100

Lay a straight reference (tape line, floor tiles, a wall edge), put the robot
on it, press f, and watch which way it drifts.

    f  drive forward for <duration> s    b  drive backward
    any key while driving stops at once
    a / z   motor A (left)  +1 / -1      A / Z  +5 / -5
    k / m   motor B (right) +1 / -1      K / M  +5 / -5
    ] / [   both +1 / -1 (overall speed, keeps the balance)
    + / -   duration +0.5 / -0.5 s
    w  save to the calibration file      q  quit

Drifts RIGHT -> left wheel (A) is faster: press z, or k.
Drifts LEFT  -> right wheel (B) is faster: press m, or a.
(If that is backwards on your robot, the motor wiring is swapped: just use
the opposite key.)

The saved file (default ~/.config/pirobot/motor_calib.json, or
$PIROBOT_MOTOR_CALIB) is read by motion_subscriber at startup.
"""
import argparse
import select
import sys
import termios
import time
import tty

from pirobot import motor_control as mc


def getkey(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def drive(move, duration):
    """Run `move` for `duration` s; any key aborts. Returns elapsed time."""
    t0 = time.monotonic()
    move()
    try:
        while time.monotonic() - t0 < duration:
            if getkey(0.02) is not None:
                break
    finally:
        mc.stopmotors()
    return time.monotonic() - t0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--duration', type=float, default=3.0, help='seconds per test run')
    ap.add_argument('--frequency', type=float, default=None,
                    help='PWM frequency in Hz (default: from calibration file, else 20)')
    ap.add_argument('--file', default=mc.CALIB_FILE, help='calibration file')
    args = ap.parse_args(argv)

    loaded = mc.load_calibration(args.file)
    if args.frequency is not None:
        mc.Frequency = args.frequency
    mc.initialize_motors(calibrate=False)
    duration = args.duration

    print(__doc__)
    print(f'Loaded {loaded}' if loaded
          else f'No calibration file yet ({args.file}); using defaults.')

    def show(extra=''):
        print(f'A={mc.DutyCycleA:5.1f}  B={mc.DutyCycleB:5.1f}  '
              f'(B-A={mc.DutyCycleB - mc.DutyCycleA:+.1f})  '
              f'freq={mc.Frequency:g} Hz  duration={duration:.1f} s  {extra}')

    steps = {'a': (1, 0), 'z': (-1, 0), 'A': (5, 0), 'Z': (-5, 0),
             'k': (0, 1), 'm': (0, -1), 'K': (0, 5), 'M': (0, -5),
             ']': (1, 1), '[': (-1, -1)}
    old = termios.tcgetattr(sys.stdin)
    dirty = False
    try:
        tty.setcbreak(sys.stdin.fileno())
        show()
        while True:
            k = getkey(0.5)
            if k is None:
                continue
            if k == 'q':
                break
            if k in steps:
                da, db = steps[k]
                mc.set_duty(mc.DutyCycleA + da, mc.DutyCycleB + db)
                dirty = True
                show()
            elif k in '+=':
                duration = min(15.0, duration + 0.5)
                show()
            elif k in '-_':
                duration = max(0.5, duration - 0.5)
                show()
            elif k in 'fb':
                print('forward...' if k == 'f' else 'backward...', end=' ', flush=True)
                t = drive(mc.forward if k == 'f' else mc.backward, duration)
                show(f'ran {t:.1f} s')
            elif k == 'w':
                show(f'saved -> {mc.save_calibration(args.file)}')
                dirty = False
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        mc.cleanup()
    if dirty:
        print('Note: last changes were NOT saved (press w to save).')
        print(f'  duty_a={mc.DutyCycleA:g} duty_b={mc.DutyCycleB:g} frequency={mc.Frequency:g}')


if __name__ == '__main__':
    main()
