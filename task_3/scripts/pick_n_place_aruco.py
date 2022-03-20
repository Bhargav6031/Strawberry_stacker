#!/usr/bin/env python3
import rospy
import time
import math
import cv2
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_ros_link_attacher.srv import Gripper
from task_1 import *


class strawdrone:
    def __init__(self):
        self.gripperState = False
        rospy.Subscriber(
            "/gripper_check", String, self.checkGripper)

    def checkGripper(self, data):
        if data.data == "True":
            print("True")
            self.gripperState = True

    def deliver(self):
        rospy.wait_for_service('/activate_gripper')
        try:
            attach = rospy.ServiceProxy(
                '/activate_gripper', Gripper)
            attach(True)
        except rospy.ServiceException as e:
            print("attach call failed: %s" % e)


		
				
	
	
	
	
class offboard_control:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('pick_n_place', anonymous=True)
        self.bridge = CvBridge()
        rospy.Subscriber(
            "/edrone/camera/image_raw", Image,self.image_callback,rospy.tcpNoDelay())
        rospy.spin()
    
    def image_callback(self, data):
        print("oiiiii")
        self.img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

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

    def setTakeoff(self):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy(
                'mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude=3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" % e)

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
    drone = strawdrone()

    # Initialize publishers
    local_pos_pub = rospy.Publisher(
        'mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher(
        'mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate
    rate = rospy.Rate(20.0)

    # Make the list of setpoints
    setpoints = [[0, 0, 3], [3, 0, 3],[6, 0, 3],[6.4, 0, 3], [6.4, 0, 0.2], [6.5, 0, -0.5], [6.5, 0, 0],[0,0,3]]
    velo = [[0, 0, 5], [1, 0, 0],[1, 0, 0],[1, 0, 0], [0, 0, 1], [0, 0, 1], [0, 0, 1],[0,0,3]]
    # Similarly initialize other publishers

    # Create empty message containers
    pos = PoseStamped()
    pos.pose.position.x = 3
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
    a = 1


    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  

        
        Write your algorithm here 
        '''
        if drone.gripperState == True and a:
            while not stateMt.state.mode == "AUTO.LAND":
                ofb_ctl.set_Auto_Land()
                rate.sleep()
            time.sleep(3)
            drone.deliver()
            time.sleep(3)
            a = 0
            while not stateMt.state.mode == "AUTO.TAKEOFF":
                ofb_ctl.setTakeoff()
                rate.sleep()
            
            
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
