#!/usr/bin/env python3

import sys
import rospy
from motor_control.srv import Esp32

def motor_command_client(left, right):
    rospy.wait_for_service('motor_command')
    try:
        motor_command = rospy.ServiceProxy('motor_command', Esp32)
        resp1 = motor_command(left, right)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
def usage():
    return "%s [left right]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        left = float(sys.argv[1])
        right = float(sys.argv[2])
        motor_command_client(left, right)
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s %s"%(left, right))
