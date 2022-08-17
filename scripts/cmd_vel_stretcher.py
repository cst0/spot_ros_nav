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
from rospy.timer import TimerEvent

class CmdVelStretcher:
    def __init__(self):
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        self.cmd_vel_pub = rospy.Publisher('spot/cmd_vel', Twist, queue_size=1)
        self.most_recent = None
        self.timer = rospy.Timer(rospy.Duration(1/30), self.timer_cb)
        self.timings = []
        self.last_time = rospy.Time.now().to_sec()

    def cmd_vel_cb(self, msg:Twist):
        self.timings.append(rospy.Time.now().to_sec() - self.last_time)
        if len(self.timings) > 10:
            self.timings.pop(0)
        self.last_time = rospy.Time.now().to_sec()

        self.most_recent = msg

    def timer_cb(self, event:TimerEvent):
        if event.last_real is None or self.most_recent is None or len(self.timings) == 0:
            # no useful data yet
            return

        avg_subscription_rate = (sum(self.timings) / len(self.timings))
        # use the TimerEvent to determine if it's been too long since we last saw a message
        if rospy.Time.now().to_sec() - self.last_time < avg_subscription_rate:
            # if so, just publish the most recent message
            self.cmd_vel_pub.publish(self.most_recent)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_stretcher')
    c = CmdVelStretcher()
    rospy.spin()
    c.cmd_vel_pub.unregister()
    c.cmd_vel_sub.unregister()
    c.timer.shutdown()
    rospy.loginfo('cmd_vel_stretcher shutting down')
