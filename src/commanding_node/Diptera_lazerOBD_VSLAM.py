#!/usr/bin/env python3.6

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Cloudlaser:

    def __init__(self):

        self.laser = float
        self.min_obs_dis = 1.3
        self.move = "MOVE"
        self.blocked = "BLOCKED"
        self.obs_msg = "Obstacle ahead"
        self.no_obs_msg = "Clear path ahead"

    def cb_lzr(self, msg):
        #
        for idx in range(120, 220, 2):
            self.laser = msg.ranges[idx]
            if self.laser < self.min_obs_dis:
                self.publish_obs(self.blocked, self.obs_msg, idx, self.laser)
            else:
                self.publish_obs(self.move, self.no_obs_msg, idx, self.laser)

    def publish_obs(self, action, msg_info, angle, distance):
        obstinfo = rospy.Publisher('/Diptera/Obstacle/info', String, queue_size=10)
        obsmsg = rospy.Publisher('/Diptera/Obstacle', String, queue_size=10)
        # cont_msg = "%s at angle %s ,time %s" % msg ,%angle %rospy.get_time()
        msg_info_const = msg_info + ", distance: " + str(float(distance)) + ", angle: " + str(angle) + ", time: " + str(
            rospy.get_time())
        obstinfo.publish(msg_info_const)
        obsmsg.publish(action)
        # self.rate = rospy.Rate(2)
        # self.rate.sleep()

    def listiner(self):
        rospy.init_node("Diptera_Obstacle_detection")
        rospy.Subscriber("/astra/scan", sensor_msgs.msg.LaserScan, self.cb_lzr)
        rospy.spin()
        # self.rate.sleep()


#if __name__ == '__main__':
cld = Cloudlaser()
cld.listiner()
