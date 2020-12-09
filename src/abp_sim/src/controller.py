#!/usr/bin/env python

import sys
import rospy
import rospkg
from math import sqrt, cos
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState
from abp_sim.srv import DesiredCoords, DesiredCoordsResponse


desired = [0., 0., 0.]
state   = [0., 0., 0.]
kp = 2.5
lock_yaw = False

def handle_desired(req):
    global desired, lock_yaw
    desired = req.data
    lock_yaw = req.lock_yaw

    #print(desired)
    res = DesiredCoordsResponse()
    res.rec = True
    return res


def get_velocity(d, x):
    global kp
    v = []
    for i in range(len(d)):
        v.append(-kp * (x[i] - d[i]))
    
    return v

def distance(d, s):
    x = s[0] - d[0]
    y = s[1] - d[1]
    return sqrt(x**2 + y**2)


def controller():
    global state, desired, lock_yaw

    name = rospy.get_namespace()
    rospy.init_node(name[1:-1] + '_controller')

    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.Service(name+'desired', DesiredCoords, handle_desired) 
    model = GetModelStateRequest()
    model.model_name = name[1:-1]

    cmd_pub = rospy.Publisher(name+'velocity', Float32MultiArray, queue_size=4)
    cmd = Float32MultiArray()
    
    desired[0] = float(sys.argv[1])
    desired[1] = float(sys.argv[2])
    desired[2] = float(sys.argv[3])

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        req = get_model_srv(model)
        rp = req.pose.position
        ra = req.pose.orientation

        ang = euler_from_quaternion([ra.x, ra.y, ra.z, ra.w])
        state = [rp.x, rp.y, ang[2]]

        if lock_yaw:
            vel = get_velocity(desired, state)
            alt = []
            alt.append(4 * vel[0] / abs(vel[0]) if vel[0] else 0.)
            alt.append(4 * vel[1] / abs(vel[1]) if vel[1] else 0.)
            alt.append(3 * vel[2] / abs(vel[2]) if vel[2] else 0.)
            vel[0] = min(vel[0], alt[0], key=abs)
            vel[1] = min(vel[1], alt[1], key=abs)
            vel[2] = min(vel[2], alt[2], key=abs)
            cmd.data = vel
            #print(desired)
        else:
            dist = distance(desired, state)
            if dist == 0.:
                dira = state[2]
            else:
                dira = cos((desired[0] - state[0]) / dist)
            if dist > 0.25 and abs(dira - state[2]) > 0.1:
                vel = get_velocity([0., 0., dira], state)
                cmd.data = [0., 0., vel[2]]
            elif dist > 0.25:
                vel = get_velocity(desired, state)
                alt = []
                alt.append(4 * vel[0] / abs(vel[0]) if vel[0] else 0.)
                alt.append(4 * vel[1] / abs(vel[1]) if vel[1] else 0.)
                alt.append(3 * vel[2] / abs(vel[2]) if vel[2] else 0.)
                vel[0] = min(vel[0], alt[0], key=abs)
                vel[1] = min(vel[1], alt[1], key=abs)
                #vel[2] = min(vel[2], alt[2], key=abs)
                vel[2] = 0.
                cmd.data = vel
            elif dist < 0.25 and abs(desired[2] - state[2]) > 0.1:
                vel = get_velocity(desired, state)
                cmd.data = [0., 0., vel[2]]

        #if name[1:-1] is 'satlet0':
        #    print(cmd.data)
        cmd_pub.publish(cmd)
        rate.sleep()



if __name__ == "__main__":
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
