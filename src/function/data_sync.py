#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage, Imu
from gps_common.msg import GPSFix
from std_msgs.msg import String
import json

# 센서 구성 파일을 읽기
with open('../sensor_config.json') as config_file:
    sensor_configs = json.load(config_file)

# 센서 데이터의 타임스탬프를 저장할 딕셔너리
sensor_timestamps = {}

# 동기화 상태를 전송하기 위한 퍼블리셔
sync_status_publisher = rospy.Publisher('/sensor_sync_status', String, queue_size=10)

# 마지막으로 메시지를 전송한 시간
last_publish_time = 0

def sensor_callback(sensor_id, data):
    global sensor_timestamps
    sensor_timestamps[sensor_id] = data.header.stamp
    check_sync()

def check_sync():
    global last_publish_time

    # 최소 4개의 센서 데이터가 필요
    if len(sensor_timestamps) < 4:
        return

    # 현재 ROS 시간
    current_time = rospy.get_time()

    # 메시지 전송 간격 (예: 5초)
    publish_interval = 5.0

    if current_time - last_publish_time < publish_interval:
        return

    # 모든 센서 타임스탬프의 최대 차이 계산
    timestamps = list(sensor_timestamps.values())
    max_diff = max(timestamps) - min(timestamps)

    # 동기화 상태와 최대 시간 차이를 JSON 형식으로 전송
    sync_data = {
        "is_synced": max_diff.to_sec() < 0.2,
        "max_time_diff": max_diff.to_sec()
    }
    sync_status_publisher.publish(json.dumps(sync_data))

    # 마지막 전송 시간 업데이트
    last_publish_time = current_time

# ROS 노드 초기화
rospy.init_node('sensor_sync_checker')

# 각 센서에 대해 콜백 함수 생성 및 구독 설정
def make_callback(sensor_id):
    def callback(data):
        sensor_callback(sensor_id, data)
    return callback

for sensor in sensor_configs:
    message_type = {
        "CompressedImage": CompressedImage,
        "GPSFix": GPSFix,
        "Imu": Imu,
        "PointCloud2": PointCloud2
    }.get(sensor['type'])

    rospy.Subscriber(sensor['topic'], message_type, make_callback(sensor['id']))

# ROS 이벤트 루프 유지
rospy.spin()
