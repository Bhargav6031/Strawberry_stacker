#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''

import rospy
import math
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(
                'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies
    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy(
                'mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def offboard_set_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
            setModeService = rospy.ServiceProxy(
                'mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")
        except rospy.ServiceException:
            print("Service takeoff call failed: ")

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure

    def set_Auto_Land(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy(
                'mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Autoland Mode could not be set." % e)


class stateMoniter:
    def __init__(self):
        self.state = State()
        self.sp = PositionTarget()
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.sp.type_mask = int('010111111000', 2)

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    # Create more callback functions for other subscribers


def main():

    stateMt = stateMoniter()
    ofb_ctl = offboard_control()
    # Initialize publishers
    local_pos_pub = rospy.Publisher(
        'mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher(
        'mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints = [[0, 0, 10], [10, 0, 10], [10, 10, 10],
                 [0, 10, 10], [0, 0,10]]  # List to setpoints
    velo = [[0, 0, 5], [5, 0, 0], [0, 5, 0], [-5, 0, 0], [0, -5, 0]]
    # Similarly initialize other publishers

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    # Similarly add other containers

    # Initialize subscriber
    rospy.Subscriber("/mavros/state", State, stateMt.stateCb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, stateMt.posCb)
    # Similarly initialize other subscribers

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")
    
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()

    # Arming the drone
    ofb_ctl.offboard_set_mode()

    # Switching the state to auto mode
    print("OFFBOARD mode activated")

    # Publish the setpoints
    i = 0
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here 
        '''
        if i >= len(setpoints):
            while not stateMt.state.mode == "AUTO.LAND":
                ofb_ctl.set_Auto_Land()
                rate.sleep()
            break

        pos.pose.position.x = setpoints[i][0]
        pos.pose.position.y = setpoints[i][1]
        pos.pose.position.z = setpoints[i][2]
        vel.linear.x = velo[i][0]
        vel.linear.y = velo[i][1]
        vel.linear.z = velo[i][2]
        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        if(abs(setpoints[i][0]-stateMt.local_pos.x) < 0.1 and abs(setpoints[i][1]-stateMt.local_pos.y) < 0.1 and abs(setpoints[i][2]-stateMt.local_pos.z) < 0.1):
            i = i+1
        rate.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
