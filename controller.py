import sys
import ast
from franky import *

def main():
    lines = sys.stdin.read().strip().splitlines()
    last_line = lines[-1].strip()
    coords = ast.literal_eval(last_line)
    x, y, z = coords

    robot = Robot("192.168.0.1")
    robot.relative_dynamics_factor = 0.01
    current_orientation = robot.current_cartesian_state.pose.rotation().as_quat()
    pose = Affine([x, y, z], current_orientation)
    motion = CartesianMotion(pose)
    print("Moving Arm")
    robot.move(motion)
    print("Motion Complete")


if __name__ == "__main__":
    main()

