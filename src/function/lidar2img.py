#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import time

class PointCloudToImage:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub1 = rospy.Publisher("/converted_image1/compressed", CompressedImage, queue_size=1)
        self.image_pub2 = rospy.Publisher("/converted_image2/compressed", CompressedImage, queue_size=1)
        self.image_pub3 = rospy.Publisher("/converted_image3/compressed", CompressedImage, queue_size=1)

        self.image_sub1 = rospy.Subscriber("/os_cloud_node/points", PointCloud2, self.callback1)
        self.image_sub2 = rospy.Subscriber("/os_cloud_node/points", PointCloud2, self.callback2)
        self.image_sub3 = rospy.Subscriber("/os_cloud_node/points", PointCloud2, self.callback3)

        self.last_time = time.time()
        self.interval = 1  # 이미지를 전송할 시간 간격 (초)

    def callback1(self, data):
        self.process_data(data, self.image_pub1)

    def callback2(self, data):
        self.process_data(data, self.image_pub2)

    def callback3(self, data):
        self.process_data(data, self.image_pub3)

    def process_data(self, data, publisher):
        current_time = time.time()
        if current_time - self.last_time >= self.interval:
            self.last_time = current_time

            # Convert PointCloud2 data to numpy array
            pc_array = pc2.read_points(data, field_names=("x", "y"), skip_nans=True)
            points = np.array(list(pc_array))
            point = np.unique(points, axis=0)

            # Convert to image
            image = self.convert_to_image(point)

            # Convert OpenCV image to ROS CompressedImage message
            try:
                ros_image = self.bridge.cv2_to_compressed_imgmsg(image, "jpg")
                publisher.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr(e)

    def convert_to_image(self, points):
        height, width = 880, 880
        image = np.zeros((height, width, 3), np.uint8)

        fixed_max_x, fixed_min_x = 15, -15
        fixed_max_y, fixed_min_y = 15, -15

        x_scaled = (points[:, 0] - fixed_min_x) / (fixed_max_x - fixed_min_x)
        y_scaled = (points[:, 1] - fixed_min_y) / (fixed_max_y - fixed_min_y)
        x_scaled = (x_scaled * width).astype(np.int32)
        y_scaled = ((1 - y_scaled) * height).astype(np.int32)

        valid_indices = (x_scaled >= 0) & (x_scaled < width) & (y_scaled >= 0) & (y_scaled < height)

        image[y_scaled[valid_indices], x_scaled[valid_indices]] = (255, 255, 255)

        return image  # Ensure to return the generated image

if __name__ == '__main__':
    rospy.init_node('point_cloud_to_image_converter', anonymous=True)
    p = PointCloudToImage()
    rospy.spin()
