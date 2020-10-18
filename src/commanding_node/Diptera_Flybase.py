#!/usr/bin/env python

import time
import tf
import mavros
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, OverrideRCIn, RCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class Vinga:

    def __init__(self):
        # publishers
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # subscribers
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.rc_in = rospy.Subscriber("/mavros/rc/in", RCIn, self.cb_rc)

        # services
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.landing = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self.pi_2 = (3.14 / 2)
        self.x = 0
        self.y = 0
        self.z = 1
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.pinpoint = None
        self.rc = RCIn()
        self.current_state = State()

    def cb_rc(self, stream):
        self.rc = stream

    def state_cb(self, state):
        # callback method for state sub
        offb_set_mode = SetMode
        self.current_state = state

    def const_message(self, x, y, z, ro, pi, yaw):
        self.pinpoint = PoseStamped()
        self.pinpoint.pose.position.x = x
        self.pinpoint.pose.position.y = y
        self.pinpoint.pose.position.z = z
        quat = tf.transformations.quaternion_from_euler(ro, pi, yaw + (3.14/2))

        self.pinpoint.pose.orientation.x = quat[0]
        self.pinpoint.pose.orientation.y = quat[1]
        self.pinpoint.pose.orientation.z = quat[2]
        self.pinpoint.pose.orientation.w = quat[3]
        return self.pinpoint

    def position_control(self):
        rospy.init_node('Diptera_offb_node', anonymous=True)
        prev_state = self.current_state
        self.rate = rospy.Rate(20.0)  # MUST be more then 2Hz

        # send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub.publish(self.const_message(self.x, self.y, self.z, self.roll, self.pitch, self.yaw))
            self.rate.sleep()

        # wait for FCU connection
        while not self.current_state.connected:
            self.rate.sleep()

        last_request = rospy.get_rostime()
        #for i in range(500):
        while not rospy.is_shutdown():
            print ("current state: %s" % self.current_state.mode)
            print ("requested position is %s" % self.pinpoint.pose)
            now = rospy.get_rostime()
            if self.current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5)):
                self.set_mode_client(base_mode=1, custom_mode="OFFBOARD")
                last_request = now
            else:
                if not self.current_state.armed and (now - last_request > rospy.Duration(5)):
                    self.arming_client(True)
                    last_request = now

                # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state.armed:
                rospy.loginfo("Vehicle armed: %r" % self.current_state.armed)
                print(self.current_state.armed)
            if prev_state.mode != self.current_state.mode:
                rospy.loginfo("Current mode: %s" % self.current_state.mode)
            prev_state = self.current_state
            #self.rate.sleep()

            self.waypoint()

            # Update timestamp and publish pose
            self.pinpoint.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.const_message(self.x, self.y, self.z, self.roll, self.pitch, self.yaw))
            # self.rate.sleep()

    def volar(self, x ,y ,z ,r ,p ,yaw ,time):
        # Update timestamp and publish pose
        for i in range(time):
            self.pinpoint.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.const_message(x, y, z, r, p, yaw))
            self.rate.sleep()
            print ("requested position is %s" % self.pinpoint.pose)



    def waypoint(self):
        if self.current_state.mode == "OFFBOARD" and self.current_state.armed == True:
            print("Enter exit for other functions")
            self.volar(0.5,0,1.2,0,0,0,400)
            self.volar(-0.5,0,1.2,0,0,0,400)
            diptera.land()
            time.sleep(3)
            diptera.disarm()
            rospy.signal_shutdown("Shutting down rospy")
            #    self.random_movement()
            #    self.move_forward()
            #    self.change_yaw()

    def random_movement(self):
        value = randint(0, 1)
        if value == 1:
            self.x = self.x + 1
        else:
            self.yaw = self.yaw + 90

    def move_forward(self):
        self.x = self.x + 1

    def change_yaw(self):
        self.yaw = self.yaw + 90

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.landing()
        print ("Diptera has landed")

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arming_client(False)


if __name__ == '__main__':
    try:
        diptera = Vinga()
        diptera.position_control()



    except rospy.ROSInterruptException:
        pass
