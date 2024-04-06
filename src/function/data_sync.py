#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Bool
import time

class SensorSyncCheckNode:
    def __init__(self):
        rospy.init_node('sensor_sync_check_node', anonymous=True)
        self.publisher = rospy.Publisher('/sensor_sync_status', Bool, queue_size=10)

        self.sensor_subscriptions = []  # No direct equivalent, but we will subscribe in the loop
        self.last_msg_times = {}

        sensors = [
            ("/a65/image_raw/compressed", CompressedImage),
            ("/blackfly/image_raw/compressed", CompressedImage),
            ("/ouster1/points", PointCloud2),
            ("/ouster2/points", PointCloud2),
            ("/ouster3/points", PointCloud2),
        ]

        for topic, msg_type in sensors:
            rospy.Subscriber(topic, msg_type, self.sensor_callback, callback_args=topic)
            self.last_msg_times[topic] = rospy.Time.now()

        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_sensor_sync)
        self.sync_status_history = []

    def sensor_callback(self, msg, topic):
        self.last_msg_times[topic] = rospy.Time.now()

    def check_sensor_sync(self, event):
        current_time = rospy.Time.now()
        time_diffs = [(current_time - self.last_msg_times[topic]).to_sec() for topic in self.last_msg_times]

        # Consider out of sync if time difference is greater than a threshold (e.g., 100ms)
        out_of_sync = any(diff > 0.1 for diff in time_diffs)  # Conversion to seconds

        sync_status = not out_of_sync
        self.sync_status_history.append(sync_status)
        if len(self.sync_status_history) > 10:
            self.sync_status_history.pop(0)

        # If False is counted 7 or more times, publish False, otherwise True
        if self.sync_status_history.count(False) >= 7:
            final_status = False
        else:
            final_status = True

        self.publisher.publish(Bool(data=final_status))

if __name__ == '__main__':
    try:
        sensor_sync_check_node = SensorSyncCheckNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Node terminated by the user')
    except rospy.ROSInterruptException:
        pass
