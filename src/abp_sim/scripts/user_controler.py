#!/usr/bin/env python

import rospy
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node('user_input')

    state_msg = ModelState()
    state_msg.model_name = 'satlet0'
    
    state_msg.twist.angular.z = 10

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

    except rospy.ServiceException, e:
        print("Service Call Failed : %s" % e)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

