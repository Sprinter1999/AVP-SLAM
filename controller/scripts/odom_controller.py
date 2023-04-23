#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry


'''
TODO: A Path following controller that drive the robot to a given sequence of x and y coordinates given in controller/configFile.yaml
'''


def Pose_callback(Odometry):
# call back function to get data from /odom topic
    global x
    global y
    global first_flag
    global x_0, y_0

    # set the x and y to 0 when the controller starts
    if first_flag == 0:
        x_0 = Odometry.pose.pose.position.x
        y_0 = Odometry.pose.pose.position.y
        first_flag = 1

    x = Odometry.pose.pose.position.x - x_0
    y = Odometry.pose.pose.position.y - y_0

def imu_callback(orientation):
# callback function to get current yaw angle
    global yaw_angle
    yaw_angle = orientation.pose.position.z

def vel_limit(vel_x):
    if vel_x > VelLimit:
        return VelLimit
    else:
        return vel_x

def rot_limit(vel_z):
    if vel_z < 0 and abs(vel_z) > rotLimit:
        return -rotLimit
    elif vel_z > 0 and abs(vel_z) > rotLimit:
        return rotLimit
    else:
        return vel_z


def pureRot(point_index):
    delta_x = (goal_x[point_index] - x)
    delta_y = (goal_y[point_index] - y)
    theta = yaw_angle
    goal_angle = math.atan2(delta_y, delta_x)*180/math.pi
    
    alpha = -theta + goal_angle

    if abs(alpha) > abs(alpha + 360):
        alpha = alpha + 360
    elif abs(alpha - 360) < abs(alpha):
        alpha = alpha - 360
    else:
        pass

    alpha = alpha * math.pi/180
    vel = Twist()
    vel.angular.z = rot_limit(k_alpha * alpha)
    global pub
    pub.publish(vel)

def control_command(point_index):
    delta_x = (goal_x[point_index] - x)
    delta_y = (goal_y[point_index] - y)
    theta = yaw_angle
    # global Ang_Dif
    # Ang_Dif = goal_theta[point_index] - theta
    global rho
    rho = math.sqrt(delta_x**2 + delta_y**2)
    goal_angle = math.atan2(delta_y, delta_x)*180/math.pi
    alpha = -theta + goal_angle
    Angle = Twist()
    Angle.linear.x = goal_angle
    Angle.linear.y = alpha
    Angle.linear.z = rho
    global pub_Angle
    pub_Angle.publish(Angle)

    # beta = goal_theta[point_index] -theta - alpha

    # wrap alpha between -pi and pi
    if abs(alpha) > abs(alpha + 360):
        alpha = alpha + 360
    elif abs(alpha - 360) < abs(alpha):
        alpha = alpha - 360
    else:
        pass
    print("X: %f, Y: %f, Theta: %f" %(x, y, theta))

    Alpha = alpha
    alpha = alpha * math.pi/180
    # beta = beta * math.pi/180

    # Phase 0 -> rotating only, Phase 1 -> rotate and translate
    global phase_flag 
    vel = Twist()
    if abs(Alpha) >= Ang_tolerance and phase_flag == 0:
        print("Phase 0")
        vel.linear.x = 0
        vel.angular.z = rot_limit(k_alpha * alpha)
    else:
        print("Phase 1")
        phase_flag = 1
        vel_x = k_rho * rho
        vel_x = vel_limit(vel_x)
        vel_x = math.cos(alpha) * vel_x
        vel.linear.x = vel_x
        vel.angular.z = rot_limit(k_alpha * alpha)

    global pub
    pub.publish(vel)

def ReachedGoal():
    rospy.loginfo("Reached Goal!!!")
    global pub
    pub.publish(Stop)

if __name__ == '__main__':
    
    # initialize pose
    x = 0.0
    y = 0.0
    yaw_angle = 0.0
    rho = 2.0 # should be initialized bigger than Dist_tolerance to get in the first loop

    # parameters for orientation control
    # Ang_Dif = 10.0
    # goal_theta = [0, 180, 0, -180]
    
    # parameters
    goal_x = rospy.get_param("x_coordinate")
    goal_y = rospy.get_param("y_coordinate")
    k_rho = 0.3
    k_alpha = -1.0 # 1.0
    k_beta = -0.3
    Dist_tolerance = 0.1
    Ang_tolerance = 2.0

    Stop = Twist()
    Stop.linear.x = 0
    Stop.angular.z = 0
    VelLimit = 0.4
    rotLimit = math.pi/20.0
    phase_flag = 0

    x_0 = 0
    y_0 = 0
    first_flag = 0
    
    point_index = 0
    goal_flag = 0


    rospy.init_node('odom_controller', anonymous=True)
    rospy.Subscriber("/odom", Odometry, Pose_callback)
    rospy.Subscriber("/yaw_odom", PoseStamped, imu_callback)
    pub = rospy.Publisher('/ideal_cmd_vel', Twist, queue_size=10)
    pub_Angle = rospy.Publisher('/Angle', Twist, queue_size=10)
    
    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():
        if goal_flag == 1:
            ReachedGoal()
            break
        elif rho <= Dist_tolerance and goal_flag == 0:
            point_index += 1 # increase the index to get the next coordinate
            phase_flag = 0
            if point_index == len(goal_x):
                goal_flag = 1
                continue
            control_command(point_index)
        else:
            control_command(point_index)
        rate.sleep()