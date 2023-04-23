#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from math import sin, cos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

'''
Generate noisy odometry based on control command in the format of x velocity "v" and angular velocity "w".
Used for wheel encoder odometry simulation.

Subscribe ideal control command from /ideal_cmd_vel which is sent from controller and publish noisy command to /cmd_vel for Gazebo simulation.
'''


def wrap2pi(yaw):
    if yaw < -np.pi:
        yaw += 2*np.pi
    elif yaw > np.pi:
        yaw -= 2*np.pi
    return yaw

def odom_callback(data):
    global vertex_pub
    global edge_pub
    global cmd_pub
    v = data.linear.x
    w = data.angular.z

    ######################################################################
    #################### calculate vel cmd for gazebo ####################
    ######################################################################

    gazebo_vel_cmd = Twist()
    g_R = np.array([[(gazebo_c1* abs(v)+gazebo_c2*abs(w))**2, 0], [0, (gazebo_c3* abs(v)+gazebo_c4*abs(w))**2]])
    g_rand = np.random.multivariate_normal([v, w], g_R)
    v_actual = g_rand[0]
    w_actual = g_rand[1]
    gazebo_vel_cmd.linear.x = v_actual
    gazebo_vel_cmd.angular.z = w_actual
    cmd_pub.publish(gazebo_vel_cmd)

    ######################################################################
    ################## ready to publish as noisy input ###################
    ######################################################################
    global previous_pose
    global id
    global pre_time
    now = rospy.get_time()
    
    dt = now - pre_time

    # reset dt if it is unreasonably large
    if dt > 0.1:
        dt = 0.01

    pre_time = now
    theta = previous_pose[2]
    
    v = v_actual
    w = w_actual
    o_R = np.array([[(odom_c1* abs(v)+odom_c2*abs(w))**2, 0], [0, (odom_c3* abs(v)+odom_c4*abs(w))**2]]) + 1e-10
    o_rand = np.random.multivariate_normal([v, w], o_R)
    v_odom = o_rand[0]
    w_odom = o_rand[1]

    # Calculating covariance based on Jacobian matrix Vt and motion noise o_R
    # Vt = np.array([[ (-sin(theta)+sin(theta+w_odom*dt))/w_odom,  v_odom*(sin(theta)-sin(theta+w_odom*dt))/(w_odom**2) + (v_odom*cos(theta+w_odom*dt)*dt)/w_odom ],
    #            [ (cos(theta)-cos(theta+w_odom*dt))/w_odom,  -v_odom*(cos(theta)-cos(theta+w_odom*dt))/(w_odom**2) + (v_odom*sin(theta+w_odom*dt)*dt)/w_odom ],
    #            [0, dt]])

    # cov = np.matmul(np.matmul(Vt, o_R), Vt.T)

    delta_v = v_odom*dt
    delta_x = v_odom*dt*np.cos(theta + w_odom*dt)
    delta_y = v_odom*dt*np.sin(theta + w_odom*dt)
    delta_theta = -w_odom * dt
    delta_pose = np.array([delta_x, delta_y, delta_theta])

    new_pose = previous_pose + delta_pose
    new_pose[2] = wrap2pi(new_pose[2])

    # Output in g2o format 
    vertex_output = "VERTEX_SE2" + " " +  str(now) + " " + str(new_pose[0]) + " " + str(new_pose[1]) + " " + str(new_pose[2])
    edge_output = "EDGE_SE2" + " " + str(pre_time) + " " + str(now) + " " + str(delta_v) + " " + str(0) + " " + str(delta_theta) \
        + " " + str(cov[0, 0]) + " " + str(cov[0, 1]) + " " + str(cov[0, 2]) + " " + str(cov[1, 1]) + " " + str(cov[1, 2]) + " " + str(cov[2, 2])
    
    previous_pose = new_pose

    # Make sure that a vertex published before edges. For the purpose of graph optimization
    if(id > 0):
        edge_pub.publish(edge_output)    
        vertex_pub.publish(vertex_output)
    else: 
        vertex_pub.publish(vertex_output)
        id += 1


if __name__ == '__main__':

    # Load parameters
    gazebo_c1 = rospy.get_param("gazebo_c1")
    gazebo_c2 = rospy.get_param("gazebo_c2")
    gazebo_c3 = rospy.get_param("gazebo_c3")
    gazebo_c4 = rospy.get_param("gazebo_c4")

    odom_c1 = rospy.get_param("odom_c1")
    odom_c2 = rospy.get_param("odom_c2")
    odom_c3 = rospy.get_param("odom_c3")
    odom_c4 = rospy.get_param("odom_c4")

    cov = rospy.get_param("odom_cov")
    cov = np.asarray(cov).reshape(3, 3)

    id = 0

    previous_pose = np.array([0, 0, 0])

    try:
        rospy.init_node('Odometry', anonymous=True)
        vertex_pub = rospy.Publisher('/vertex_odom', String, queue_size=10)
        edge_pub = rospy.Publisher('/edge_odom', String, queue_size=10)
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pre_time = rospy.get_time()
        # subscribe ideal control commands publish noise ones to /cmd_vel topic
        rospy.Subscriber("/ideal_cmd_vel", Twist, odom_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass