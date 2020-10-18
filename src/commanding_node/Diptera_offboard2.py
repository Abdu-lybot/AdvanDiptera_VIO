#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import math
import threading
import yaml



# callback method for state sub
#current_state = State()
#offb_set_mode = SetMode

class Vamos:


    def __init__(self):
        local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        state_sub = rospy.Subscriber("/mavros/state", State, state_cb)
        arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)
        local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_pose_callback)

    def state_cb(self, state):
        self.current_state = state

    def position_control(self):
        rospy.init_node('offb_node', anonymous=True)
        prev_state = self.current_state
        rate = rospy.Rate(20.0)  # MUST be more then 2Hz

        for i in range(100): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
                self.cur_target_pose = construct_target(0, 0, 0, current_heading)

        # send a few setpoints before starting
        for i in range(10):
            local_target_pub.publish(self.cur_target_pose)
            offboard_state = offboard()
            rate.sleep()
            arm_state = arm() # Arms the drone

        while not rospy.is_shutdown():
            local_target_pub.publish(cur_target_pose)
            time.sleep(0.1)

    def imu_callback(self, msg):
        self.imu = msg
        self.current_heading = q2yaw(imu.orientation)


    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad


    def construct_target(self, x, y, z, yaw, yaw_rate = 1):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 9

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose

    def offboard(self):
        if set_mode_client(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vehicle Offboard failed")
            rospy.loginfo("Vehicle Offboard failed!")
            return False


    def arm(self):
        if arming_client(True):
            rospy.loginfo("Drone armed!")
            return True
        else:
            print("Vehicle arming failed!")
            rospy.logerr("Drone arming failed!")
            return False

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg



if __name__ == '__main__':
    try:
        v = Vamos()
        v.position_control()
    except rospy.ROSInterruptException:
        pass
