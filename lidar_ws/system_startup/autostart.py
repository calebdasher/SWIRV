import Jetson.GPIO as GPIO
import rospy
import roslaunch
import subprocess
import time
import shlex
import sys
import signal
import psutil
import asyncio

# does not yet work
# can't figure out how to launch roscore and then continue to the rest of the program to run startup.launch

ros_channel = 32 # pin GPIO 12 on breakout board
wifi_channel = 36
GPIO.cleanup()
GPIO.setmode(GPIO.BOARD)
GPIO.setup(ros_channel, GPIO.IN)
GPIO.add_event_detect(ros_channel, GPIO.BOTH) # detect rising and falling edges

GPIO.setup(wifi_channel, GPIO.IN)
GPIO.add_event_detect(wifi_channel, GPIO.BOTH)

subprocess.Popen(['roscore'], start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
time.sleep(3)

rospy.init_node('start', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, options_wait_for_master=False)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/jetson/Desktop/lidar_ws/src/startup/launch/startup.launch"])

while True:
    if GPIO.event_detected(ros_channel):
        print("Detected input")
        if GPIO.input(ros_channel) == GPIO.HIGH:
            print("High")
            launch.start()
        if GPIO.input(ros_channel) == GPIO.LOW:
            launch.shutdown()
            roscore.terminate()
    if GPIO.event_detected(wifi_channel):
        if GPIO.input(wifi_channel) == GPIO.HIGH:
            print("starting SWIRV_AP")
            subprocess.call(['sh', 'restart_wifi_as_ap.sh'])
        if GPIO.input(wifi_channel) == GPIO.LOW:
            print("Connecting to WCU-Guest")
            subprocess.call(['sh', 'stop_ap_and_switch_to_client.sh'])

        

