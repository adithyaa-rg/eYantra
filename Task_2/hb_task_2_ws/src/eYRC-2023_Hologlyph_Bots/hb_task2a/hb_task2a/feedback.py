#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
       
from std_msgs.msg import String
from sensor_msgs.msg import Image            
import numpy as np
import random
import time
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
# Import the required modules
##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        self.bridge = CvBridge()
        camera_subscriber = self.create_subscription(Image,'/camera/image_raw',self.image_callback,10)
        camera_subscriber  # prevent unused variable warning

        # Subscribe the topic /camera/image_raw


    def image_callback(self, msg):
        # self.get_logger().info(f'Image received {msg.height}x{msg.width}')
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f'Image converted : {cv_image.shape}') # 500x500x3
           

            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
            parameters =  aruco.DetectorParameters_create()

            try:
                corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
                
            except:
                print("Error in Aruco Detection")

            for i,j in zip(corners,ids):
                average_x = i.T[0].mean()
                average_y = i.T[1].mean()
                theta = np.arctan2(average_y, average_x)
                # print(
                #     f"ID = {j} \n",
                #     f"Average X = {average_x} \n",
                #     f"Average Y = {average_y} \n",
                #     f"Theta = {theta} \n"
                # )

                new_image = cv2.putText(
                img = cv_image,
                text = f"id = {j[0]}",
                org = (int(average_x), int(average_y)),
                fontFace = cv2.FONT_HERSHEY_PLAIN,
                fontScale = 1,
                color = (0, 255, 0),
                thickness = 1
                )
                
                pts = np.array(i[0], np.int32)
                pts = pts.reshape((-1,1,2))
                new_image = cv2.polylines(new_image,
                                          [pts],            # Points
                                          True,             # isClosed
                                          (0, 255, 0))      # Colour                # Thickness
                
                cv2.imshow("Image window", new_image)
                cv2.waitKey(3)
                
        except CvBridgeError as e:
            print(e)
        

        #convert ROS image to opencv image
        #Detect Aruco marker
        # Publish the bot coordinates to the topic  /detected_aruco

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()