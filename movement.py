import sys
import termios
import tty
from franky import *

def get_key():
    keyboard = sys.stdin.fileno()
    settings = termios.tcgetattr(keyboard)
    try:
        tty.setraw(keyboard)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(keyboard, termios.TCSADRAIN, settings)
    return ch

def main():
    robot = Robot("192.168.0.1")
    gripper = Gripper("192.168.0.1")
    speed = 0.02
    step = 0.01
    force = 20.0
    gripper.move(0.01,speed)
    width = 0.01
    gstep = 0.01
    robot.relative_dynamics_factor = 0.05
    while True:
        key = get_key().lower()

        if key == 'q':
            print("Exit")
            break

        elif key == 's':
            move = CartesianMotion(Affine([step, 0.0, 0.0]), ReferenceType.Relative)
            robot.move(move)
        elif key == 'w':
            move = CartesianMotion(Affine([-step, 0.0, 0.0]), ReferenceType.Relative)
            robot.move(move)
        elif key == 'a':
            move = CartesianMotion(Affine([0.0, step, 0.0]), ReferenceType.Relative)
            robot.move(move)
        elif key == 'd':
            move = CartesianMotion(Affine([0.0, -step, 0.0]), ReferenceType.Relative)
            robot.move(move)
        elif key == 'l':
            move = CartesianMotion(Affine([0.0, 0.0, step]), ReferenceType.Relative)
            robot.move(move)
        elif key == 'p':
            move = CartesianMotion(Affine([0.0, 0.0, -step]), ReferenceType.Relative)
            robot.move(move)
        elif key == 'h':
            gripper.move((width+gstep),speed)
            width = width + gstep
        elif key == 'j':
            gripper.move((width-gstep),speed)
            width = width - gstep
        else:
            continue
if __name__ == "__main__":
    main()
