#!/usr/bin/env python3


'''
* Team Id : SS_2202
* Author List : Bhargav M Gowda
* Filename: SS_2202_strawberry_stacker.py
* Theme: Strawberry Stacker
* Functions: checkGripper,attach_gripper,detach_gripper,image_callback,spawn_clb,setArm,offboard_set_mode,
             stateCb,posCb,velCb,reachvel,caminfo_callback,reachpos,deliver
* Global Variables: [b,r,posi,arr]
'''

'''
LOGIC : Drone1 scans the markers based on row numbers and sends it through global queue and drone2 will do the delivery of
boxes based on the data given by drone1
'''

import rospy
import cv2
import time
import math
import numpy as np
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String,UInt8
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from gazebo_ros_link_attacher.srv import Gripper
from task_1 import *
from tf import transformations as tr

import cv2.aruco as aruco        
                
from numpy import sqrt 
import multiprocessing

from multiprocessing import Process
    
    


class offboard_control:

    def __init__(self,num,temp):
        rospy.init_node('strawberry_stacker', anonymous=True)

        self.rate = rospy.Rate(20.0)

        #temp_4 stores whether a drone requires access to add aruco marker location
        self.temp_4=temp

        #stores the setpoints
        self.setpoints=[]

        self.pos = PoseStamped()
        self.pos.pose.position.x = 0
        self.pos.pose.position.y = 0
        self.pos.pose.position.z = 0


        #no of drone
        self.dronenum=num

        #empty image
        self.img=np.ones((512,512,3))


        #different paths necessary for subsrcibing the topics or services
        self.cam_path = "/edrone" + str(self.dronenum)+"/camera/image_raw"
        self.arm_path = "/edrone" + str(self.dronenum)+"/mavros/cmd/arming"
        self.mode_path = "/edrone" + str(self.dronenum)+"/mavros/set_mode"
        self.takeoff_path = "/edrone" + \
            str(self.dronenum)+"/mavros/cmd/takeoff"
        self.land_path = "/edrone" + str(self.dronenum)+"/mavros/cmd/land"


        #initialising the variable which checks whether gripper is going to attach or not
        self.gripperState = False

        #subscribing to a topic which checks whether gripper is going to attach or not
        rospy.Subscriber(
            "/edrone" + str(self.dronenum)+"/gripper_check", String, self.checkGripper)


        #publishing the local position
        self.local_pos_pub = rospy.Publisher(
        '/edrone'+str(self.dronenum)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)


        #publishing the local velocity
        self.local_vel_pub = rospy.Publisher(
        "/edrone"+str(self.dronenum)+"/mavros/setpoint_velocity/cmd_vel" , TwistStamped , queue_size=10)

        

        self.state = State()

        #initialising the local position variable
        self.local_pos = Point(0.0, 0.0, 0.0)

        #initialising the local veloctiy variable
        self.local_vel = TwistStamped()

        #initialising the empty dictionary which holds the id and the corners.
        self.aruco_list = {}

        self.pos3=0
        self.temp_3=-1

        #initialising the id variable
        self.id=-1
        self.setp=[]


        #location of the cells in blue truck and red truck

        if self.dronenum==1:
            self.blue=[[[15, -68.4, 5],[15, -68.4,6],[15,-68.4,4],[15,-68.4,2],[15,-68.4,1.6 ],[15,-68.4,5]],
              [[15, -67.17, 5],[15, -67.17,6],[15,-67.17,4],[15,-67.17,2],[15,-67.17,1.6 ],[15,-67.17,5]],
              [[15, -65.94, 5],[15, -65.94,6],[15,-65.94,4],[15,-65.94,2],[15,-65.94,1.6 ],[15,-65.94,5]],
              [[15.7, -8.4, 5],[14.85, -8.4,6],[14.85,-8.4,4],[14.85,-8.4,2],[14.85,-8.4,1.6 ],[14.85,-8.4,5]],
              [[15.7, -7.17, 5],[14.85, -7.17,6],[14.85,-7.17,4],[14.85,-7.17,2],[14.85,-7.17,1.6 ],[14.85,-7.17,5]],
              [[15.7, -5.94, 5],[14.85, -5.94,6],[14.85,-5.94,4],[14.85,-5.94,2],[14.85,-5.94,1.6 ],[14.85,-5.94,5]],
              [[16.55, -8.4, 5],[14.85, -8.4,6],[14.85,-8.4,4],[14.85,-8.4,2],[14.85,-8.4,1.6 ],[14.85,-8.4,5]],
              [[16.55, -7.17, 5],[14.85, -7.17,6],[14.85,-7.17,4],[14.85,-7.17,2],[14.85,-7.17,1.6 ],[14.85,-7.17,5]],
              [[14.55, -5.94, 5],[14.85, -5.94,6],[14.85,-5.94,4],[14.85,-5.94,2],[14.85,-5.94,1.6 ],[14.85,-5.94,5]],
              ]
            self.red=[ [[57.8, 3.75, 6],[57.8, 3.75, 5],[57.8, 3.75, 4],[57.8, 3.75, 2],[57.8, 3.75, 1.6],[57.8, 3.75, 6]],
              [[57.8, 5, 5],[57.5, 5, 6],[57.5, 5, 4],[57.5, 5, 2],[57.5, 5, 1.6],[57.5, 5, 5]],
              [[57.8, 6.25, 5],[57.5, 6.25, 6],[57.5, 6.25, 4],[57.5, 6.25, 2],[57.5, 6.25, 1.6],[57.5, 6.25, 5]],
              [[58.35, 3.75, 5],[57.5, 3.75, 6],[57.5, 3.75, 4],[57.5, 3.75, 2],[57.5, 3.75, 1.6],[57.5, 3.75, 5]],
              [[58.35, 5, 5],[57.5, 5, 6],[57.5, 5, 4],[57.5, 5, 2],[57.5, 5, 1.6],[57.5, 5, 5]],
              [[58.35, 6.25, 5],[57.5, 6.25, 6],[57.5, 6.25, 4],[57.5, 6.25, 2],[57.5, 6.25, 1.6],[57.5, 6.25, 5]],
              [[59.2, 65.75, 5],[57.5, 65.75, 6],[57.5, 65.75, 4],[57.5, 65.75, 2],[57.5, 65.75, 1.6],[57.5, 65.75, 5]],
              [[59.2, 67, 5],[57.5, 67, 6],[57.5, 67, 4],[57.5, 67, 2],[57.5, 67, 1.6],[57.5, 67, 5]],
              [[59.2, 68.25, 5],[57.5, 68.25, 6],[57.5, 68.25, 4],[57.5, 68.25, 2],[57.5, 68.25, 1.6],[57.5, 68.25, 5]],
              ]
              
        if self.dronenum==0:
            self.blue=[[[15, -8.4, 5],[15, -8.4,6],[15,-8.4,4],[15,-8.4,2],[15,-8.4,1.6 ],[15,-8.4,5]],
              [[15, -7.17, 5],[15, -7.17,6],[15,-7.17,4],[15,-7.17,2],[15,-7.17,1.6 ],[15,-7.17,5]],
              [[15, -5.94, 5],[15, -5.94,6],[15,-5.94,4],[15,-5.94,2],[15,-5.94,1.6 ],[15,-5.94,5]],
              [[15.7, -8.4, 5],[14.85, -8.4,6],[14.85,-8.4,4],[14.85,-8.4,2],[14.85,-8.4,1.6 ],[14.85,-8.4,5]],
              [[15.7, -7.17, 5],[14.85, -7.17,6],[14.85,-7.17,4],[14.85,-7.17,2],[14.85,-7.17,1.6 ],[14.85,-7.17,5]],
              [[15.7, -5.94, 5],[14.85, -5.94,6],[14.85,-5.94,4],[14.85,-5.94,2],[14.85,-5.94,1.6 ],[14.85,-5.94,5]],
              [[16.55, -8.4, 5],[14.85, -8.4,6],[14.85,-8.4,4],[14.85,-8.4,2],[14.85,-8.4,1.6 ],[14.85,-8.4,5]],
              [[16.55, -7.17, 5],[14.85, -7.17,6],[14.85,-7.17,4],[14.85,-7.17,2],[14.85,-7.17,1.6 ],[14.85,-7.17,5]],
              [[14.55, -5.94, 5],[14.85, -5.94,6],[14.85,-5.94,4],[14.85,-5.94,2],[14.85,-5.94,1.6 ],[14.85,-5.94,5]],
              ]
            self.red=[ [[57.8, 3.75, 6],[57.8, 3.75, 5],[57.8, 3.75, 4],[57.8, 3.75, 2],[57.8, 3.75, 1.6],[57.8, 3.75, 6]],
              [[57.8, 5, 5],[57.5, 5, 6],[57.5, 5, 4],[57.5, 5, 2],[57.5, 5, 1.6],[57.5, 5, 5]],
              [[57.8, 6.25, 5],[57.5, 6.25, 6],[57.5, 6.25, 4],[57.5, 6.25, 2],[57.5, 6.25, 1.6],[57.5, 6.25, 5]],
              [[58.35, 3.75, 5],[57.5, 3.75, 6],[57.5, 3.75, 4],[57.5, 3.75, 2],[57.5, 3.75, 1.6],[57.5, 3.75, 5]],
              [[58.35, 5, 5],[57.5, 5, 6],[57.5, 5, 4],[57.5, 5, 2],[57.5, 5, 1.6],[57.5, 5, 5]],
              [[58.35, 6.25, 5],[57.5, 6.25, 6],[57.5, 6.25, 4],[57.5, 6.25, 2],[57.5, 6.25, 1.6],[57.5, 6.25, 5]],
              [[59.2, 65.75, 5],[57.5, 65.75, 6],[57.5, 65.75, 4],[57.5, 65.75, 2],[57.5, 65.75, 1.6],[57.5, 65.75, 5]],
              [[59.2, 67, 5],[57.5, 67, 6],[57.5, 67, 4],[57.5, 67, 2],[57.5, 67, 1.6],[57.5, 67, 5]],
              [[59.2, 68.25, 5],[57.5, 68.25, 6],[57.5, 68.25, 4],[57.5, 68.25, 2],[57.5, 68.25, 1.6],[57.5, 68.25, 5]],
              ]

     

        self.marker_length=0.135

        self.CAMERA_MATRIX = None
        self.DISTORTION_COEFFICIENTS = None

        self.have_cam_info = False
        
        self.bridge = CvBridge()
        self.te=0.0

        
        #subscribing to the camera
        rospy.Subscriber(
            self.cam_path, Image,self.image_callback)
        rospy.Subscriber(
            "/edrone" + str(self.dronenum)+"/camera/camera_info", CameraInfo, callback=self.caminfo_callback)
        self.vel_msg=TwistStamped()




    '''   
    * Function Name: checkGripper
    * Input: data(msg)
    * Output: returns whether the gripper can be attached or not
    * Logic: checks whether the gripper can be attached or not
    * Example Call: checkGripper
    '''
    def checkGripper(self, data):
        #returns true if gripper can attach to the box
        if data.data == "True":
            self.gripperState = True
        else:
            self.gripperState = False





    '''   
    * Function Name: detach_gripper
    * Input: None
    * Output: attaches the gripper
    * Logic: attaches the gripper
    * Example Call: attach_gripper()
    '''
    def attach_gripper(self):
        #attaches the gripper to the box
        rospy.wait_for_service("/edrone" + str(self.dronenum)+"/activate_gripper")
        try:
            attach = rospy.ServiceProxy(
                "/edrone" + str(self.dronenum)+"/activate_gripper", Gripper)
            attach(True)
        except rospy.ServiceException as e:
            print("attach call failed: %s" % e)




    '''   
    * Function Name: detach_gripper
    * Input: None
    * Output: detaches the gripper
    * Logic: detaches the gripper
    * Example Call: detach_gripper()
    '''
    def detach_gripper(self):
        #detaches the gripper to the box
        rospy.wait_for_service("/edrone" + str(self.dronenum)+"/activate_gripper")
        try:
            attach = rospy.ServiceProxy(
                "/edrone" + str(self.dronenum)+"/activate_gripper", Gripper)
            attach(False)
        except rospy.ServiceException as e:
            print("attach call failed: %s" % e)



    '''   
    * Function Name: image_callback
    * Input: data
    * Output: adds the position of the detected marker into the queue(posi)
    * Logic: detect the corner, id and postion of the detected the marker when the angle between the camera and aruco marker
             turns from positive to negetive 
    * Example Call: image_callback
    '''
    def image_callback(self, data):
        try:
            self.aruco_list={}

            #converting data to a cv2 standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")

            #converting the image into gray colour for better detection
            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

            aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
            parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(
                gray, aruco_dict, parameters=parameters)

            #we have to know about the new row number updated everytime so it should be called frequently.
            #since image_callback function is called frequently we have places it here and calling it two times isn't 
            #necessary so data from either of class is sufficient. 
            if self.dronenum==0:
                rospy.Subscriber("/spawn_info",UInt8,callback=self.spawn_clb)
            
            
            #only if the corners detected which means aruco marker present enters this condition
            if len(corners):
                self.id=ids[0][0]
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length, self.CAMERA_MATRIX, self.DISTORTION_COEFFICIENTS)
                rotMat = tr.euler_matrix(np.pi / 2.0, 0, 0)
                rotMat = rotMat[0:3, 0:3]
                tvecs = np.matmul(rotMat, tvecs[0][0].T)

                #when the angle between the camera and aruco marker turns from positive to negetive we have aruco marker exacly 
                #below us it is enough to know the local position in x and y of the drone which will be the position pf aruco marker
                #since only drone 1 is doing the scanning it is sufficient to activate the detection for only drone 1
                if self.temp_4:
                    if tvecs[0]<0 and self.te>0:
                        posi.put([self.local_pos.x,self.local_pos.y])
                        posi.put(self.id)

                    self.te=tvecs[0]
                

                for k in range(len(corners)):
                    temp_1 = corners[k]
                    temp_1 = temp_1[0]
                    temp_2 = ids[k]
                    temp_2 = temp_2[0]
                    self.aruco_list[temp_2] = temp_1

        except CvBridgeError as e:
            print(e)





    '''   
    * Function Name: spawn_clb
    * Input: msg(data)
    * Output: row number of the boxes
    * Logic: adds the row number into the queue(arr)
    * Example Call: spawn_clb
    '''
    def spawn_clb(self, data):
        sp=data.data
        if (self.temp_3!=sp):
            arr.put(sp)
        self.temp_3=sp





    '''   
    * Function Name: setArm
    * Input: None
    * Output: arms the drone
    * Logic: arms the drone
    * Example Call: setArm()
    '''
    def setArm(self):
        rospy.wait_for_service(self.arm_path)
        try:
            armService = rospy.ServiceProxy(
                self.arm_path, mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)




    '''   
    * Function Name: offboard_set_mode
    * Input: None
    * Output: changes to offboard mode
    * Logic: changes to offboard mode
    * Example Call: offboard_set_mode()
    '''
    def offboard_set_mode(self):
        rospy.wait_for_service(self.mode_path)
        try:
            setModeService = rospy.ServiceProxy(
                self.mode_path, mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")
        except rospy.ServiceException:
            print("Service takeoff call failed: ")




    '''   
    * Function Name: posCb
    * Input: msg
    * Output: local position
    * Logic:  callback function required for local position
    * Example Call: posCb(msg)
    '''
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z



        (r, p, y) = tr.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.pos3 = y



    '''   
    * Function Name: velCb
    * Input: msg
    * Output: local velocity
    * Logic:  callback function required for local velocity
    * Example Call: velCb(msg)
    '''
    def velCb(self, msg):
        
        self.local_vel=msg




    '''   
    * Function Name: reachvel
    * Input: self.setpoints(holds the velocity setpoints we have to travel) in required time
    * Output: offboard navigation
    * Logic: offborad navigation using velocity setpoints
    * Example Call: reachpos(x,y,z,time)
    '''
    def reachvel(self,x,y,z,time):
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


            self.local_vel_pub.publish(self.vel_msg)
            self.rate.sleep()





    '''   
    * Function Name: caminfo_callback
    * Input: caminfo_msg
    * Output: distortion coefficients and camera matrix
    * Logic:  callback function required for camera caliberation
    * Example Call: caminfo_callback(caminfo_msg)
    '''
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




    '''   
    * Function Name: reachpos
    * Input: self.setpoints(holds the setpoints we have to travel)
    * Output: offboard navigation
    * Logic: offborad navigation using postion setpoints
    * Example Call: reachpos()
    '''

    def reachpos(self):

        for i in range (0,len(self.setpoints)):


            self.pos.pose.position.x = self.setpoints[i][0]
            self.pos.pose.position.y = self.setpoints[i][1]
            self.pos.pose.position.z = self.setpoints[i][2]
        

            while not (abs(self.setpoints[i][0]-self.local_pos.x) < 0.1 and abs(self.setpoints[i][1]-self.local_pos.y) < 0.1 and abs(self.setpoints[i][2]-self.local_pos.z) < 0.1):
                self.local_pos_pub.publish(self.pos)

            self.rate.sleep()





    '''   
    * Function Name: deliver
    * Input: position_of_detected_marker,id_of_detected_marker
    * Output: None
    * Logic: "We add the setpoints consisting the position of the detected marker to setp list and depending upon the detected id
              we add the setpoints consisting the position of the the exact cell we have to deliver the box and then we navigate
              to the setpoints we added and deliver the box " 
    * Example Call: deliver(position_of_detected_marker,id_of_detected_marker,b,r)
    '''
    def deliver(self,position_of_detected_marker,id_of_detected_marker,blue_cell,red_cell):
        #For the sake of simplicity we call "position_of_detected_marker" as "posit" and "id_of_detected_marker" as "posid"

        self.setp=[]
        t1=self.local_pos.x
        t2=self.local_pos.y
        self.setp.append([t1,t2,5])
        if self.dronenum==1:
            self.setp.append([posit[0],posit[1]-60,5])
            self.setp.append([posit[0],posit[1]-60,1])
            self.setp.append([posit[0],posit[1]-60,0])
            self.setp.append([posit[0],posit[1]-60,-0.12])
            self.setp.append([posit[0],posit[1]-60,5])
        if self.dronenum==0:
            self.setp.append([posit[0],posit[1],5])
            self.setp.append([posit[0],posit[1],1])
            self.setp.append([posit[0],posit[1],-0.11])
            self.setp.append([posit[0],posit[1],5])

        if posid==1:
            self.setpoints=self.red[red_cell.value]
            red_cell.value=red_cell.value+1
        elif posid ==2:
            self.setpoints=self.blue[blue_cell.value]
            blue_cell.value=blue_cell.value+1


        self.setp=self.setp+self.setpoints
        f=0
        for i in range (0,len(self.setp)):
            self.pos.pose.position.x = self.setp[i][0]
            self.pos.pose.position.y = self.setp[i][1]
            self.pos.pose.position.z = self.setp[i][2]
            a=0.1

            while not (abs(self.setp[i][0]-self.local_pos.x) < a and abs(self.setp[i][1]-self.local_pos.y) < a and abs(self.setp[i][2]-self.local_pos.z) < a):
                
                if self.gripperState==True and i<7:
                    self.attach_gripper()
                    self.rate.sleep()

                if self.local_pos.z < 1.9  and i>7 :
                    self.detach_gripper()
                    self.rate.sleep()



                self.local_pos_pub.publish(self.pos)

            self.rate.sleep()







#func1 controls only drone no 1 

def func1(posi,arr,blue_cell,red_cell):
    ofb_ctl = offboard_control(0,1)



    ofb_ctl.setpoints=[[0,0,3]]
    



    rospy.Subscriber("/edrone"+str(ofb_ctl.dronenum)+"/mavros/local_position/pose", PoseStamped, ofb_ctl.posCb)
    rospy.Subscriber("/edrone"+str(ofb_ctl.dronenum)+"/mavros/local_position/velocity_local", TwistStamped, ofb_ctl.velCb)
    
  
    while not ofb_ctl.state.armed:
        ofb_ctl.setArm()
        ofb_ctl.rate.sleep()
    print("Armed!!")

    for i in range(100):
        ofb_ctl.local_pos_pub.publish(ofb_ctl.pos)
        ofb_ctl.rate.sleep()
    
    
    ofb_ctl.offboard_set_mode()

    ofb_ctl.reachpos()
    i=0
    while i<30:
        row=arr.get()
        ofb_ctl.setpoints=[[0,(row-1)*4-0.3,3],[0,(row-1)*4+0.2,3],[0,(row-1)*4-0.1,3],[0,(row-1)*4,3]]
        ofb_ctl.reachpos()
        ofb_ctl.reachvel(2,0,0,3)

       

    

    




#func2 controls only drone no 2


def func2(posi,arr,blue_cell,red_cell):
    ofb_ctl = offboard_control(1,0)

    


    rospy.Subscriber("/edrone"+str(ofb_ctl.dronenum)+"/mavros/local_position/pose", PoseStamped, ofb_ctl.posCb)
    
    spawn=rospy.Subscriber("/spawn_info",UInt8)



    a=0.1
    c=1

    while c:
        if posi.qsize()>0:
            while not ofb_ctl.state.armed:
                ofb_ctl.setArm()
                ofb_ctl.rate.sleep()
            print("Armed!!")

            for i in range(100):
                ofb_ctl.local_pos_pub.publish(ofb_ctl.pos)
                ofb_ctl.rate.sleep()
            
            
            ofb_ctl.offboard_set_mode()
            while True :
                position_of_detected_marker=posi.get()
                id_of_detected_marker=posi.get()
                ofb_ctl.deliver(position_of_detected_marker,id_of_detected_marker,blue_cell,red_cell)
             







if __name__ == '__main__':
    

    try:
        #current blue cell
        blue_cell=multiprocessing.Value('i',0)

        #current red cell
        red_cell=multiprocessing.Value('i',0)

        #stores the positions of aruco markers detected
        posi= multiprocessing.Queue()

        #stores the row numbers spawned
        arr=multiprocessing.Queue()


        p1 = Process(target = func1,args=(posi,arr,blue_cell,red_cell,))
        p1.start()
        p2 = Process(target = func2,args=(posi,arr,blue_cell,red_cell,))
        p2.start()
    except rospy.ROSInterruptException:
        pass
