#!/usr/bin/python
# coding=UTF-8
import math
import threading
from utilis import *
from geometry_msgs.msg import PoseStamped
from nlink_parser.msg import LinktrackNodeframe2
from nlink_parser.msg import TofsenseFrame0
from sensor_msgs.msg import Imu


class ExPositionHandler:
    def __init__(self):
        self.cur_uwb_position = [0., 0., 0.]
        self.cur_tof_dis = 0.0
        self.cur_height = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        rospy.init_node("ExPositionHandler", anonymous=True)
        rospy.loginfo("succeed to initial node %s", rospy.get_name())
        rospy.Subscriber("nlink_linktrack_nodeframe2", LinktrackNodeframe2, self.uwb_callback, queue_size=10)
        rospy.Subscriber("nlink_tofsense_frame0", TofsenseFrame0, self.tof_sense_callback, queue_size=10)
        rospy.Subscriber("mavros/imu/data", Imu, self.imu_callback, queue_size=10)
        self.vision_pub = rospy.Publisher("mavros/vision_pose/pose", PoseStamped, queue_size=100)
        spin_thread = threading.Thread(target=spin_job)
        spin_thread.setDaemon(True)
        spin_thread.start()

    def uwb_callback(self, uwb_msg):
        self.cur_uwb_position = uwb_msg.pos_3d

    def tof_sense_callback(self, tof_sense_msg):
        self.cur_tof_dis = tof_sense_msg.dis
        self.update_cur_height()
        # print("[%.3f, %.3f， %.3f],dis %.3f,height %.3f" % (self.roll / math.pi * 360, self.pitch / math.pi * 360,
        #                                                    self.yaw / math.pi * 360, self.cur_tof_dis, self.cur_height))

    def imu_callback(self, imu):
        euler = quaternion_to_euler([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])
        self.roll, self.pitch, self.yaw = euler
        self.update_cur_height()
        # print("[%.3f, %.3f， %.3f],dis %.3f,height %.3f" % (self.roll / math.pi * 360, self.pitch / math.pi * 360,
        #                                                    self.yaw / math.pi * 360, self.cur_tof_dis, self.cur_height))

    def update_cur_height(self):
        self.cur_height = get_height(self.cur_tof_dis, self.roll, self.pitch)

    def handle_vision_position(self):
        while not rospy.is_shutdown():
            vision_msg = PoseStamped()
            vision_msg.header.stamp = rospy.get_rostime()
            vision_msg.pose.position.x = self.cur_uwb_position[0]
            vision_msg.pose.position.y = self.cur_uwb_position[1]
            vision_msg.pose.position.z = self.cur_height
            self.vision_pub.publish(vision_msg)
            sleep_duration = rospy.Duration.from_sec(1 / 50)  # 50HZ
            rospy.sleep(sleep_duration)


if __name__ == "__main__":
    h = ExPositionHandler()
    h.handle_vision_position()
