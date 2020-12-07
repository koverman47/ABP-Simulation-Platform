#!/usr/bin/env python

import sys
import rospy
import rospkg
from math import sqrt
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelStates
from abp_sim.srv import FloatSrv, FloatSrvRequest


class Planner():

    def __init__(self):
        #self.objects = {}
        self.opp = [0., 0., 0.]
        self.state = [0., 0., 0.]
        self.name = None
        self.get_model_srv = None
        self.des_srv = None
        sub = None
        self.id = int(sys.argv[4])

    def init(self):
        name = rospy.get_namespace()
        self.name = name[1:-1]
        rospy.init_node(self.name + '_planner')

        #rospy.wait_for_service('/gazebo/get_model_state')
        #self.get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        rospy.wait_for_service('/'+name+'/desired')
        self.des_srv = rospy.ServiceProxy('/'+name+'/desired', FloatSrv)

    def state_callback(self, msg):
        ind = msg.name.index(self.name)
        rp = msg.pose[ind].position
        ra = msg.pose[ind].orientation
        ang = euler_from_quaternion([ra.x, ra.y, ra.z, ra.w])
        self.state = [rp.x, rp.y, ang[2]]
        #if self.id == 0:
        #    ind = msg.name.index('satlet1')
        #else:
        #    ind = msg.name.index('satlet0')
        #rp = msg.pose[ind].position
        #ra = msg.pose[ind].orientation
        #ang = euler_from_quaternion([ra.x, ra.y, ra.z, ra.w])
        #self.opp = [rp.x, rp.y, ang[2]]
        


    def distance(self, u, v):
        return sqrt((u[0] - v[0])**2 + (u[1] - v[1])**2)

    def main(self):
        self.init()
        name = rospy.get_namespace()
        self.name = name[1:-1]
        rospy.init_node(self.name+'_planner')

        off_x = float(sys.argv[1])
        off_y = float(sys.argv[2])
        off_yaw = float(sys.argv[3])
        desired = [off_x, off_y, off_yaw]

        waypoints = waypoints = []
        waypoints.append([0., 0., 0.])
        waypoints.append([4., 0., 1.])
        waypoints.append([3., 2., 3.141592654])
        waypoints.append([3., 3., 1.5707])
        waypoints.append([2., 0., 0.])

        stability = 0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            dist = self.distance(desired, self.state)
            if dist < 0.5:
                stability += 1
            else:
                stability = 0
            if dist < 0.5 and waypoints and stability > 5:
                wp = waypoints.pop()
                desired[0] = wp[0] + off_x
                desired[1] = wp[1] + off_y
                desired[2] = wp[2] + off_yaw
            des = FloatSrvRequest()
            des.data = desired
            req = self.des_srv(des)
            rate.sleep()

    
if __name__ == "__main__":
    try:
        planner = Planner()
        planner.main()
    except rospy.ROSInterruptException:
        pass
