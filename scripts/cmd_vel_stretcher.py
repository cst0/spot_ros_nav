#!/usr/bin/env python3

"""
the spot has a very high cutoff for input command velocity rates:
    if they don't come in fast enough the robot will stutter at a
    high speed. to work around that, this script will subscribe
    to the cmd_vel topic and republish it at a much higher
    rate on the /spot/cmd_vel topic.
"""

import rospy
from geometry_msgs.msg import Twist

class CmdVelStretcher:
    def __init__(self):
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        self.cmd_vel_pub = rospy.Publisher('spot/cmd_vel', Twist, queue_size=1)
        self.most_recent = None
        self.timer = rospy.Timer(rospy.Duration(1/40), self.timer_cb)
        self.timings = []
        self.last_time = rospy.Time.now()

    def cmd_vel_cb(self, msg:Twist):
        #if abs(msg.linear.x) < 0.1 \
        #    and abs(msg.linear.y) < 0.1 \
        #    and abs(msg.angular.z) < 0.1:
        #    return

        self.timings.append(rospy.Time.now() - self.last_time)
        if len(self.timings) > 10:
            self.timings.pop(0)
        self.last_time = rospy.Time.now()

        self.most_recent = msg

    def timer_cb(self, _):
        # check the average times
        if len(self.timings) == 0:
            return

        self.cmd_vel_pub.publish(self.most_recent)
        #avg = sum(self.timings) / len(self.timings)
        ## if the last time is within 20% of the average, then we're good to publish
        #if self.last_time - rospy.Time.now() > avg * 0.8:
        #    if self.most_recent is not None:
        #        self.cmd_vel_pub.publish(self.most_recent)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_stretcher')
    c = CmdVelStretcher()
    rospy.spin()
    c.cmd_vel_pub.unregister()
    c.cmd_vel_sub.unregister()
    c.timer.shutdown()
    rospy.loginfo('cmd_vel_stretcher shutting down')
