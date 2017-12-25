#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
import cv2
import tf
from hector_uav_msgs.srv import EnableMotors
import numpy as np
import threading
import time

class Quad:
    def __init__(self, number):
        self.name = "/uav" + str(number)
        self.control_msg = self.name + '/command/pose'
        self.camera_msg = self.name + '/front_cam/camera/image/compressed'
        print(self.name+' start\n')
        self.pub = rospy.Publisher(self.control_msg, PoseStamped, queue_size=5)
        self.poseSub = rospy.Subscriber("chatter", PoseStamped, self.getPos)
        self.service = rospy.ServiceProxy(self.name + "/enable_motors", EnableMotors)
        self.service(True)
        self.mode = 'takeoff'
        self.pose = PoseStamped()
        self.controller = PoseStamped()
        self.controller.header.frame_id = self.name.strip('/') +'/world'
        self.sp_x = number  #setpoints
        self.sp_y = number
        self.sp_z = 5
        self.sp_yaw = 0
        self.sp_w = 1
        self.pub.publish(self.controller)

    def SetMode(self,mode):
        pass

    def getPos(self,Pos):
        self.pose = Pos

    def pub_control(self):
        self.controller.pose.position.x = self.sp_x
        self.controller.pose.position.y = self.sp_y
        self.controller.pose.position.z = self.sp_z
        self.controller.pose.orientation.z = self.sp_yaw
        self.controller.pose.orientation.w = self.sp_w
        self.pub.publish(self.controller)

    def SetPose(self, x=None, y=None, z=None, yaw=None):
        if x is not None:
            self.sp_x = x
        if y is not None:
            self.sp_y = y
        if z is not None:
            self.sp_z = z
        if yaw is not None:
            self.sp_yaw = tf.transformations.quaternion_from_euler(0, 0, yaw)[2]
            self.sp_w = tf.transformations.quaternion_from_euler(0, 0, yaw)[3]

    def CalcuPose(self):
        pass

    def getready(self,request):
        return True


class Swarm:
    def __init__(self,number,Detector=None):
        self.quads = []

        for i in range(number):
            self.quads.append(Quad(i+1))

        self.quadCameras={}
        for quads in self.quads:
            rospy.Subscriber(quads.camera_msg, CompressedImage, self.ImageGet)
            self.quadCameras[quads.name+'/front_cam_optical_frame']=0

        self.CameraThread = threading.Thread(target=self.CameraShow)
        self.CameraThread.start()

        if Detector !=None:
            self.Detector=Detector
            print('Swarm detector is online\n')

    def ImageGet(self,raw_image):
        np_arr = np.fromstring(raw_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.quadCameras['/'+raw_image.header.frame_id] = image_np

    def CameraShow(self):
        while(1):
            for name,image in self.quadCameras.items():
                cv2.imshow(name,image)
            if cv2.waitKey(20) & 0xFF == ord('q'):
                break
        return

    def ImageHandle(self,image):
        pass

    def TrajectorTest(self):
        if not self.quads:
            map(self.StaticSetTakeoff,self.quads)

    @staticmethod
    def SetTakeoff(quad):
        quad.sp_z = 3
