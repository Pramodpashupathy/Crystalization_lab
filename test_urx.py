import time
import urx
from robotiq_gripper import Robotiq_Two_Finger_Gripper

ROBOT_IP = "192.168.1.102"

print("Connecting to UR robot...")
robot = urx.Robot(ROBOT_IP)
gripper = Robotiq_Two_Finger_Gripper(robot)

try:
    print("Activating gripper...")
    gripper.gripper_activate()
    time.sleep(2)

    print("Closing gripper...")
    gripper.close_gripper()
    time.sleep(2)

    print("Opening gripper...")
    gripper.open_gripper()
    time.sleep(2)

    print("Half-close gripper...")
    gripper.pos_gripper(50)
    time.sleep(2)

finally:
    print("Stopping and disconnecting...")
    robot.stopj()
    robot.close()
