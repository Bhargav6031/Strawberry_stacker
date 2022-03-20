#!/usr/bin/env python3
import rospy
import time
import math
import numpy as np
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from gazebo_ros_link_attacher.srv import Gripper
from task_1 import *
import cv2
import cv2.aruco as aruco        
                
    
    
    
    
class offboard_control:

    def __init__(self,num):
        rospy.init_node('pick_n_place', anonymous=True)

        self.rate = rospy.Rate(20.0)

        self.dronenum=num
        self.img=np.ones((512,512,3))
        self.cam_path = "/edrone" + str(self.dronenum)+"/camera/image_raw"
        self.arm_path = "/edrone" + str(self.dronenum)+"/mavros/cmd/arming"
        self.mode_path = "/edrone" + str(self.dronenum)+"/mavros/set_mode"
        self.takeoff_path = "/edrone" + \
            str(self.dronenum)+"/mavros/cmd/takeoff"
        self.land_path = "/edrone" + str(self.dronenum)+"/mavros/cmd/land"
        rospy.init_node('pick_n_place', anonymous=True)

        self.gripperState = False

        self.local_pos_pub = rospy.Publisher(
        '/edrone'+str(self.dronenum)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.local_vel_pub = rospy.Publisher(
        "/edrone"+str(self.dronenum)+"/mavros/setpoint_velocity/cmd_vel" , TwistStamped , queue_size=10)

        rospy.Subscriber(
            "/edrone" + str(self.dronenum)+"/gripper_check", String, self.checkGripper)

        self.state = State()
        self.local_pos = Point(0.0, 0.0, 0.0)

        # self.pos = PoseStamped()
        # self.pos.header = Header()
        # self.pos.header.frame_id = "base_footprint"
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.sp = PositionTarget()
        self.local_pos = Point(0.0, 0.0, 0.0)
        self.sp.type_mask = int('010111111000', 2)
        self.posi=[]
        self.aruco_list = {}

        self.zDist=0

        self.marker_length=0.6

        self.CAMERA_MATRIX = None
        self.DISTORTION_COEFFICIENTS = None

        
        
        self.bridge = CvBridge()
        rospy.Subscriber(
           self.cam_path, Image,self.image_callback)

        rospy.Subscriber(
            "/edrone" + str(self.dronenum)+"/camera/camera_info", CameraInfo, callback=self.caminfo_callback)
        self.vel_msg=TwistStamped()

    def caminfo_callback(self, caminfo_msg):

        if self.have_cam_info:
            pass
        else:
            if caminfo_msg.K == [0, 0, 0, 0, 0, 0, 0, 0, 0]:
                rospy.logwarn("CameraInfo message has K matrix all zeros")
            else:
                self.DISTORTION_COEFFICIENTS = caminfo_msg.D
                self.CAMERA_MATRIX = np.zeros((3, 3))

                for i in range(0, 3):
                    for j in range(0, 3):
                        self.CAMERA_MATRIX[i][j] = caminfo_msg.K[i * 3 + j]

                self.have_cam_info = True
    

    def checkGripper(self, data):
        if data.data == "True":
            self.gripperState = True
        else:
            self.gripperState = False

    def attach_gripper(self):
        rospy.wait_for_service("/edrone" + str(self.dronenum)+"/activate_gripper")
        try:
            attach = rospy.ServiceProxy(
                "/edrone" + str(self.dronenum)+"/activate_gripper", Gripper)
            attach(True)
        except rospy.ServiceException as e:
            print("attach call failed: %s" % e)

    def detach_gripper(self):
        rospy.wait_for_service("/edrone" + str(self.dronenum)+"/activate_gripper")
        try:
            attach = rospy.ServiceProxy(
                "/edrone" + str(self.dronenum)+"/activate_gripper", Gripper)
            attach(False)
        except rospy.ServiceException as e:
            print("attach call failed: %s" % e)
    
    def image_callback(self, data):
        try:
            self.aruco_list={}
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(
                gray, aruco_dict, parameters=parameters)
            if len(corners):
                print(str(abs( self.zDist)))
                for k in range(len(corners)):
                    temp_1 = corners[k]
                    temp_1 = temp_1[0]
                    temp_2 = ids[k]
                    temp_2 = temp_2[0]
                    self.aruco_list[temp_2] = temp_1




        except CvBridgeError as e:
            print(e)

    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        # Waiting untill the service starts
        rospy.wait_for_service(self.arm_path)
        try:
            armService = rospy.ServiceProxy(
                self.arm_path, mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)

        # Similarly delacre other service proxies
    def setDisarm(self):
        rospy.wait_for_service(self.arm_path)
        try:
            armService = rospy.ServiceProxy(
                self.arm_path, mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Service disarming call failed: %s" % e)

    def offboard_set_mode(self):
        rospy.wait_for_service(self.mode_path)
        try:
            # setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.set_mode.request.custom_mode)
            setModeService = rospy.ServiceProxy(
                self.mode_path, mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")
        except rospy.ServiceException:
            print("Service takeoff call failed: ")

    def setTakeoff(self):
        rospy.wait_for_service(self.takeoff_path)
        try:
            takeoffService = rospy.ServiceProxy(
                self.takeoff_path, mavros_msgs.srv.CommandTOL)
            takeoffService(altitude=3)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" % e)

    def set_Auto_Land(self):
        rospy.wait_for_service(self.land_path )
        try:
            flightModeService = rospy.ServiceProxy(
                self.land_path , mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. Autoland Mode could not be set." % e)

    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    def velocity_callback(self , msg):
        self.actual = msg

    def reachit(self,x,y,z,time):
        duration =time
        number_of_iteration = duration*20.0
        i=0
        while i<number_of_iteration :
            i=i+1
            self.vel_msg.twist.linear.x =x
            self.vel_msg.twist.linear.y =y
            self.vel_msg.twist.linear.z =z
            self.vel_msg.twist.angular.x = 0
            self.vel_msg.twist.angular.y = 0
            self.vel_msg.twist.angular.z =0

            if self.aruco_list:
                print(self.local_pos.x)

            self.local_vel_pub.publish(self.vel_msg)
            self.rate.sleep()




def main():

    ofb_ctl = offboard_control(0)

+   sepoints=[[0,0,3],[2.32,0,3],[2.32,0,3.1]]
    

    # setpoints = [[0, 0, 3], [10 ,0, 3], [9.5, 0,3], [9.85, 0, 3],[9.95,0,1],[10,0,0.1],[10 , 0, -0.13], [10, 0,0],[10,0,6],[15,-8.4,6], [15,-8.4, 3],[15,-8.4, 2],[15,-8.4,1.6 ],[15,-8.4,3],
    #                 [15,-8.4,6],[0,0,0]]

    
    pos = PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0



    rospy.Subscriber("/edrone"+str(ofb_ctl.dronenum)+"/mavros/state", State, ofb_ctl.stateCb)
    rospy.Subscriber("/edrone"+str(ofb_ctl.dronenum)+"/mavros/local_position/pose", PoseStamped, ofb_ctl.posCb)
  
    while not ofb_ctl.state.armed:
        ofb_ctl.setArm()
        ofb_ctl.rate.sleep()
    print("Armed!!")

    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()
    
    # for i in range(100):

    #     ofb_ctl.vel_msg.twist.linear.x =0
    #     ofb_ctl.vel_msg.twist.linear.y =0
    #     ofb_ctl.vel_msg.twist.linear.z =-0.1
    #     ofb_ctl.vel_msg.twist.angular.x = 0
    #     ofb_ctl.vel_msg.twist.angular.y = 0
    #     ofb_ctl.vel_msg.twist.angular.z =0

    #     ofb_ctl.local_vel_pub.publish(ofb_ctl.vel_msg)
    #     ofb_ctl.rate.sleep()

    ofb_ctl.offboard_set_mode()



    for i in range (0,len(setpoints)):
            
            
        pos.pose.position.x = setpoints[i][0]
        pos.pose.position.y = setpoints[i][1]
        pos.pose.position.z = setpoints[i][2]
        
        print("    "   + str(i))

        while not (abs(setpoints[i][0]-ofb_ctl.local_pos.x) < a and abs(setpoints[i][1]-ofb_ctl.local_pos.y) < a and abs(setpoints[i][2]-ofb_ctl.local_pos.z) < a):
            # if ofb_ctl.gripperState==True:
                
            #     ofb_ctl.attach_gripper()
            #     rate.sleep()

            # if ofb_ctl.local_pos.z < 1.9 and i>9:
            #     print(setpoints[i])
            #     ofb_ctl.detach_gripper()
            #     rate.sleep()

            if a==0.1:
                a=0.0001
                
            local_pos_pub.publish(pos)

        rate.sleep()




    # print("OFFBOARD mode activated")
    # ofb_ctl.reachit(0,0,4,5)
    # ofb_ctl.reachit(3,0,0,10)
    
# mix of velocity setpoint for scanning and position setpoint for delivery 





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
