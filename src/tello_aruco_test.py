#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
<<<<<<< HEAD
import time
=======
import ffff
xxx


ggggggggg

lllllllllllll

vvvvvvvvvvvvvvvvvvv
# for taking pictures
# imag_counter = 0

# marker_size = 10
# calib_path = ""
# camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter = ',')
# camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter = ',')
# R_flip = np.zeros((3, 3), dtype = np.float32)
# R_flip[0, 0] = 1
# R_flip[1, 1] = -1
# R_flip[2, 2] = -1
# font = cv2.FONT_HERSHEY_PLAIN

# pub_x = rospy.Publisher("/x", Float32, queue_size=10)
# pub_y = rospy.Publisher("/y", Float32, queue_size=10)
# pub_z = rospy.Publisher("/z", Float32, queue_size=10)
# pub_roll = rospy.Publisher("/roll", Float32, queue_size=10)
# pub_pitch = rospy.Publisher("/pitch", Float32, queue_size=10)
# pub_yaw = rospy.Publisher("/yaw", Float32, queue_size=10)

# def isRotationMatrix(R):
#     Rt = np.transpose(R)
#     shouldBeIdentity = np.dot(Rt, R)
#     I = np.identity(3, dtype = R.dtype)
#     n = np.linalg.norm(I - shouldBeIdentity)
#     return n < 1e-6

# def rotationMatrixToEulerAngles(R):
#     assert(isRotationMatrix(R))

#     sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

#     singular = sy < 1e-6

#     if not singular:
#         x = math.atan2(R[2, 1], R[2, 2])
#         y = math.atan2(-R[2, 0], sy)
#         z = math.atan2(R[1, 0], R[0, 0])
#     else:
#         x = math.atan2(-R[1, 2], R[1, 1])
#         y = math.atan2(-R[2, 0], sy)
#         z = 0

#     return np.array([x, y, z])
>>>>>>> 5e21ac83d0b623a3c87994a714c79ab5be0d76c2

def convert_color_image(ros_image):
    bridge = CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        cv2.namedWindow("Color")
        cv2.imshow("Color", color_image)
        k = cv2.waitKey(10)
        if k % 256 == 32:
            time_name = str(time.time())
            img_name = time_name + ".png"
            cv2.imwrite(img_name, color_image)
            # imag_counter = imag_counter + 1
    except CvBridgeError as e:
        print(e)

def tellodetect():
    rospy.init_node("tello_aruco_test", anonymous=True)
    rospy.Subscriber("tello/raw_image", Image, callback=convert_color_image, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        tellodetect()
    except rospy.ROSInterruptException:
        pass


# <node pkg="image_transport" name="image_compressed" type="republish" args="raw in:=image_raw compressed out:=image_raw" />
# <node pkg="image_transport" name="image_compressed1" type="republish" args="h264 in:=image_raw raw out:=raw_image" />