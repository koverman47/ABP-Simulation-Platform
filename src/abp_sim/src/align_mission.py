#!/usr/bin/env python

import sys
import rospy
import rospkg
from math import sqrt, cos, sin
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelStates
from abp_sim.srv import DesiredCoords, DesiredCoordsRequest


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

        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        rospy.wait_for_service('/'+name+'/desired')
        self.des_srv = rospy.ServiceProxy('/'+name+'/desired', DesiredCoords)

    def state_callback(self, msg):
        ind = msg.name.index(self.name)
        rp = msg.pose[ind].position
        ra = msg.pose[ind].orientation
        ang = euler_from_quaternion([ra.x, ra.y, ra.z, ra.w])
        self.state = [rp.x, rp.y, ang[2]]
        if self.id == 0:
            ind = msg.name.index('satlet1')
        else:
            ind = msg.name.index('satlet0')
        rp = msg.pose[ind].position
        ra = msg.pose[ind].orientation
        ang = euler_from_quaternion([ra.x, ra.y, ra.z, ra.w])
        self.opp = [rp.x, rp.y, ang[2]] 


    def distance(self, u, v):
        #return sqrt((u[0] - v[0])**2 + (u[1] - v[1])**2)
        x = u[0] - v[0]
        y = u[1] - v[1]
        yaw = u[2] - v[2]
        return sqrt(x**2 + y**2 + yaw**2)
    
    def model_distance(self, u, v):
        d1 = abs(u[0] - v[0]) - 1
        d2 = abs(u[1] - v[1]) - 1
        return sqrt(d1**2 + d2**2)

    def main(self):
        self.init()
        name = rospy.get_namespace()
        self.name = name[1:-1]
        rospy.init_node(self.name+'_planner')

        off_x = float(sys.argv[1])
        off_y = float(sys.argv[2])
        off_yaw = float(sys.argv[3])
        desired = [off_x, off_y, off_yaw]

        waypoints = []
        if self.id == 1:
            r = 3
            for i in range(10):
                interval = (2 * 3.141592654) / 10
                point = [r * cos(interval * i), r * sin(interval * i), 0]
                waypoints.append(point)
        print(waypoints)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            dist = self.distance(desired, self.state)
            hypo = self.distance(self.opp, self.state)
            if hypo == 0.:
                dira = 0
            else:
                dira = cos((self.opp[0] - self.state[0]) / hypo)
            temp = None
            if abs(dira - self.state[2]) > 0.1:
                temp = [desired[0], desired[1]]
                desired[0] = self.state[0]
                desired[1] = self.state[1]
                desired[2] = dira
            elif dist < 0.25 and waypoints:
                wp = waypoints.pop()
                #desired[0] = wp[0] + off_x
                #desired[1] = wp[1] + off_y
                desired[0] = wp[0]
                desired[1] = wp[1]
                desired[2] = dira
            else:
                desired[0] = self.state[0]
                desired[1] = self.state[1]
                desired[2] = dira
            des = DesiredCoordsRequest()
            des.data = desired
            des.lock_yaw = True
            req = self.des_srv(des)
            if temp:
                desired[0] = temp[0]
                desired[1] = temp[1]
            rate.sleep()

    
if __name__ == "__main__":
    try:
        planner = Planner()
        planner.main()
    except rospy.ROSInterruptException:
        pass
