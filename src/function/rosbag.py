#!/usr/bin/env python3
import rospy
import subprocess
import os
import glob
from std_msgs.msg import String, Bool
import datetime

path = "/home/dataset"
rosbag_process = None

def check_rosbag_status():
    global path
    list_of_files = glob.glob(path + '/*.bag.active')
    return True if list_of_files else False

def get_rosbag_size():
    global path
    list_of_files = glob.glob(path + '/*.bag.active')
    if not list_of_files:
        return "0 GB"

    latest_file = max(list_of_files, key=os.path.getctime)
    size = os.path.getsize(latest_file)
    size_gb = size / (1024 * 1024 * 1024)
    return f"{size_gb:.3f} GB"

def rosbag_record(data):
    global rosbag_process
    if data.data == "ON":
        if rosbag_process is not None:
            subprocess.call(["pkill", "-f", "rosbag record"])
            rosbag_process = None
        rosbag_process = subprocess.Popen(["/home/Web/sensor_monitoring/src/function/record.sh"])
    elif data.data == "OFF":
        if rosbag_process:
            subprocess.call(["pkill", "-f", "rosbag record"])
            rosbag_process = None

def listener():
    rospy.Subscriber('/rosbag_record', String, rosbag_record)

def main():
    rospy.init_node('rosbag_control_node', anonymous=True)
    listener()

    status_pub = rospy.Publisher('/rosbag_status', Bool, queue_size=10)
    size_pub = rospy.Publisher('/rosbag_size', String, queue_size=10)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        status = check_rosbag_status()
        size = get_rosbag_size()
        status_pub.publish(status)
        size_pub.publish(size)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
