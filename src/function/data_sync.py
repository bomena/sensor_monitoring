#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import String
import json

# 센서 데이터 타임스탬프 저장
sensor_data_timestamps = {'camera1': 0, 'camera2': 0, 'lidar': 0}

def callback_camera1(data):
    sensor_data_timestamps['camera1'] = rospy.get_time()

def callback_camera2(data):
    sensor_data_timestamps['camera2'] = rospy.get_time()

def callback_lidar(data):
    sensor_data_timestamps['lidar'] = rospy.get_time()

# 동기화 상태를 전송하기 위한 퍼블리셔
sync_status_publisher = rospy.Publisher('/sensor_sync_status', String, queue_size=10)

def check_sync(event):
    # 모든 센서 데이터 간의 최대 시간 차이 확인
    timestamps = list(sensor_data_timestamps.values())
    max_time_diff = max(timestamps) - min(timestamps)
    
    # 동기화 상태와 최대 시간 차이를 JSON 형식으로 전송
    sync_data = {
        "is_synced": max_time_diff < 0.05,
        "max_time_diff": max_time_diff
    }
    sync_status_publisher.publish(json.dumps(sync_data))

def listener():
    rospy.init_node('sensor_sync_checker', anonymous=True)

    ########################### MODIFY ##################################
    rospy.Subscriber("/ouster/points", PointCloud2, callback_lidar)
    rospy.Subscriber("/cam1/image_color/compressed", CompressedImage, callback_camera1)
    rospy.Subscriber("/cam2/image_color/compressed", CompressedImage, callback_camera2)

    # rospy.Subscriber("/os_cloud_node/points", PointCloud2, callback_lidar)
    # rospy.Subscriber("/color1/image_color/compressed", CompressedImage, callback_camera1)
    # rospy.Subscriber("/color2/image_color/compressed", CompressedImage, callback_camera2)
    #####################################################################

    rospy.Timer(rospy.Duration(5), check_sync)
    rospy.spin()

if __name__ == '__main__':
    listener()
