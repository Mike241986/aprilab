#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


def chatter_callback(message):
    #get_caller_id(): Get fully resolved name of local node
    rospy.loginfo(rospy.get_caller_id() + f"I heard x = {message.point.x}, y = {message.point.y}")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("nmea_lla", PointStamped, chatter_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    #this function is sufficient for getting the data 

if __name__ == '__main__':
    listener()
