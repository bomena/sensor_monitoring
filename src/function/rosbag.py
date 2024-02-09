#!/usr/bin/env python3
import rospy
import subprocess
import os
import glob
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import String
import datetime

path = "/home/dataset"
rosbag_process = None

# Rosbag 녹음 상태 확인
def check_rosbag_status():
    global path
    list_of_files = glob.glob(path + '/*.bag.active')
    return "yes" if list_of_files else "no"

# Rosbag 파일 크기 확인
def get_rosbag_size():
    global path
    list_of_files = glob.glob(path + '/*.bag.active')
    if not list_of_files:
        return "0 GB"

    latest_file = max(list_of_files, key=os.path.getctime)
    size = os.path.getsize(latest_file)

    # Bytes를 MB로 변환
    size_mb = size / (1024 * 1024) * 0.001
    return f"{size_mb:.3f} GB"

def main():
    rospy.init_node('rosbag_control_node')

    # 서비스, 퍼블리셔 설정
    status_pub = rospy.Publisher('/rosbag_status', String, queue_size=10)
    size_pub = rospy.Publisher('/rosbag_size', String, queue_size=10)

    rate = rospy.Rate(0.05)  # 1 Hz
    while not rospy.is_shutdown():
        # Rosbag 상태 및 크기 정보 전송
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
