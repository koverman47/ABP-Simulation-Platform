#!/usr/bin/env python

import sys
import rospy
import rospkg
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.msg import ModelState
from abp_sim.srv import FloatSrv, FloatSrvResponse


desired = [0., 0., 0.]
state   = [0., 0., 0.]
kp = 0.75

def handle_desired(req):
    global desired
    desired = req.data

    print(desired)
    res = FloatSrvResponse()
    res.rec = True
    return res


def get_velocity(d, x):
    global kp
    v = []
    for i in range(len(d)):
        v.append(-kp * (x[i] - d[i]))
    
    return v


def controller():
    global state, desired

    name = rospy.get_namespace()
    rospy.init_node(name[1:-1] + '_controller')

    #state_sub = rospy.Subscriber('/gazebo/model_state', ModelState, state_callback)
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rospy.Service(name+'desired', FloatSrv, handle_desired) 
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

        vel = get_velocity(desired, state)
        alt = []
        alt.append(2 * vel[0] / abs(vel[0]) if vel[0] else 0.)
        alt.append(2 * vel[1] / abs(vel[1]) if vel[1] else 0.)
        alt.append(1 * vel[2] / abs(vel[2]) if vel[2] else 0.)
        vel[0] = min(vel[0], alt[0], key=abs)
        vel[1] = min(vel[1], alt[1], key=abs)
        vel[2] = min(vel[2], alt[2], key=abs)
        cmd.data = vel
        print(vel)

        cmd_pub.publish(cmd)
        rate.sleep()



if __name__ == "__main__":
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
