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
        self.image_pub = rospy.Publisher("/converted_image/compressed", CompressedImage, queue_size=1)

        ################################ MODIFY ####################################
        rospy.Subscriber("/ouster/points", PointCloud2, self.callback)
        ############################################################################

        self.last_time = time.time()
        self.interval = 1  # 이미지를 전송할 시간 간격 (초)

    def callback(self, data):
        current_time = time.time()
        if current_time - self.last_time >= self.interval:
            self.last_time = current_time

            # PointCloud2 데이터를 numpy 배열로 변환
            pc_array = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(pc_array))

            # 이미지 변환
            image = self.convert_to_image(points)

            # OpenCV 이미지를 ROS CompressedImage 메시지로 변환
            try:
                ros_image = self.bridge.cv2_to_compressed_imgmsg(image)
                self.image_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr(e)

    def convert_to_image(self, points):
        height = 411
        width = 880
        image = np.zeros((height, width, 3), np.uint8)

        # Define fixed scale factors based on expected range of x and y values
        fixed_max_x = 40  # Adjust based on expected max value of x
        fixed_min_x = -40  # Adjust based on expected min value of x
        fixed_max_y = 20  # Adjust based on expected max value of y
        fixed_min_y = -20  # Adjust based on expected min value of y

        for point in points:
            # Use fixed scale factors for conversion
            x_scaled = int(((point[0] - fixed_min_x) / (fixed_max_x - fixed_min_x)) * width)
            y_scaled = int(((point[1] - fixed_min_y) / (fixed_max_y - fixed_min_y)) * height)
            y_scaled = height - y_scaled  # Adjust for image coordinate system

            if 0 <= x_scaled < width and 0 <= y_scaled < height:
                image[y_scaled, x_scaled] = (255, 255, 255)  # White point

        return image



if __name__ == '__main__':
    rospy.init_node('point_cloud_to_image_converter', anonymous=True)
    p = PointCloudToImage()
    rospy.spin()
