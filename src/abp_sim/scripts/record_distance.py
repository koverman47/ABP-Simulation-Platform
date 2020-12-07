#!/usr/bin/env python

import sys
import rospy
import rospkg
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

s1 = [None, None, None]
s2= [None, None, None]


def record():
    rospy.init_node('abp_recorder_dist')

    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    m1 = GetModelStateRequest()
    m2 = GetModelStateRequest()
    m1.model_name = 'satlet0'
    m2.model_name = 'satlet1'

    f = open('logs/states.log', 'w')
    global truth, estimate, update
    rate = rospy.Rate(1)
    counter = 0
    while not rospy.is_shutdown():
        req = get_model_srv(m1)
        rp = req.pose.position
        ra = req.pose.orientation
        ang = euler_from_quaternion([ra.x, ra.y, ra.z, ra.w])
        s1 = [rp.x, rp.y, ang[2]]

        req = get_model_srv(m2)
        rp = req.pose.position
        ra = req.pose.orientation
        ang = euler_from_quaternion([ra.x, ra.y, ra.z, ra.w])
        s2 = [rp.x, rp.y, ang[2]]
        f.write("%d,%s,t\n" % (counter, ",".join(map(str, s1))))
        f.write("%d,%s,e\n" % (counter, ",".join(map(str, s2)))) 
        counter += 1

        rate.sleep()

    f.close()


if __name__ == "__main__":
    try:
        record()
    except rospy.ROSInterruptException:
        pass
