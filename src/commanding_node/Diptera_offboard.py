#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State ,OverrideRCIn ,RCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time


class Vinga():

    # callback method for state sub
    current_state = State()
    offb_set_mode = SetMode

    def __init__(self):
        #publishers
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        #subscribers
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.rc_in = rospy.Subscriber("/mavros/rc/in", RCIn ,self.cb_rc)

        #services
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.landing = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self.x = 0
        self.y = 0
        self.z = 1
        self.pinpoint = None

    def cb_rc(self, stream):
        self.rc = RCIn()
        self.rc = stream

    def state_cb(self,state):
        global current_state
        current_state = state

    def waypoint(self,x,y,z):
        self.pinpoint = PoseStamped()
        self.pinpoint.pose.position.x = x
        self.pinpoint.pose.position.y = y
        self.pinpoint.pose.position.z = z
        return self.pinpoint

    def position_control(self):
        rospy.init_node('offb_node', anonymous=True)
        prev_state = current_state
        self.rate = rospy.Rate(20.0)  # MUST be more then 2Hz

        # send a few setpoints before starting
        for i in range(100):
            self.local_pos_pub.publish(self.waypoint(self.x,self.y,self.z))
            self.rate.sleep()

        # wait for FCU connection
        while not current_state.connected:
            self.rate.sleep()

        last_request = rospy.get_rostime()
        for i in range (100):
            now = rospy.get_rostime()
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5)):
                self.set_mode_client(base_mode=1, custom_mode="OFFBOARD")
                last_request = now
            else:
                if not current_state.armed and (now - last_request > rospy.Duration(5)):
                    self.arming_client(True)
                    last_request = now

                # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != current_state.armed:
                rospy.loginfo("Vehicle armed: %r" % current_state.armed)
            if prev_state.mode != current_state.mode:
                rospy.loginfo("Current mode: %s" % current_state.mode)
            prev_state = current_state

            # Update timestamp and publish pose
            self.pinpoint.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.waypoint(self.x,self.y,self.z))
            self.rate.sleep()

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
        print ("bird is hovering")
        rospy.sleep(1)
        diptera.land()
        time.sleep(3)
        diptera.disarm()

    except rospy.ROSInterruptException:
        pass