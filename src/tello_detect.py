#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import cv2
import cv2.aruco as aruco
import math
from std_msgs.msg import Float32

# for taking pictures
# imag_counter = 0

marker_size = 10
calib_path = ""
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter = ',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter = ',')
R_flip = np.zeros((3, 3), dtype = np.float32)
R_flip[0, 0] = 1
R_flip[1, 1] = -1
R_flip[2, 2] = -1
font = cv2.FONT_HERSHEY_PLAIN

pub_x = rospy.Publisher("/x", Float32, queue_size=10)
pub_y = rospy.Publisher("/y", Float32, queue_size=10)
pub_z = rospy.Publisher("/z", Float32, queue_size=10)
pub_roll = rospy.Publisher("/roll", Float32, queue_size=10)
pub_pitch = rospy.Publisher("/pitch", Float32, queue_size=10)
pub_yaw = rospy.Publisher("/yaw", Float32, queue_size=10)

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def convert_color_image(ros_image):
    # global imag_counter
    global pub_x, pub_y, pub_z, pub_roll, pub_pitch, pub_yaw
    bridge = CvBridge()
    try:
        color_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters = parameters)
        # aruco.detectMarkers(gray_image, aruco_dict, corners, ids, )

        # print(corners)
        # print(ids)
        # print("\n")

        if len(corners) > 0:
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            # print(tvec)
            # print("\n")

            aruco.drawAxis(color_image, camera_matrix, camera_distortion, rvec, tvec, 10)
            # corners = np.array(corners)
            # corners = corners.reshape((4, 2))
            # (topLeft, topRight, bottomRight, bottomLeft) = corners
            # topRight = (int(topRight[0]), int(topRight[1]))
            # bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            # bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            # topLeft = (int(topLeft[0]), int(topLeft[1]))

            # cv2.line(color_image, topLeft, topRight, (0, 255, 0), 2)
            # cv2.line(color_image, topRight, bottomRight, (0, 255, 0), 2)
            # cv2.line(color_image, bottomRight, bottomLeft, (0, 255, 0), 2)
            # cv2.line(color_image, bottomLeft, topLeft, (0, 255, 0), 2)
            # cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            # cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            # cv2.circle(color_image, (cX, cY), 4, (0, 0, 255), -1)

            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            pos_camera = -R_tc * np.matrix(tvec).T

            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)
            roll_camera = math.degrees(roll_camera)
            pitch_camera = math.degrees(pitch_camera)
            yaw_camera = math.degrees(yaw_camera)

            str_position = "CAMERA Position x=%4.0f y=%4.0f z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
            str_attitude = "CAMERA Attitude r=%4.0f p=%4.0f y=%4.0f"%(roll_camera, pitch_camera, yaw_camera)
            cv2.putText(color_image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(color_image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            pub_x.publish(pos_camera[0])
            pub_y.publish(pos_camera[1])
            pub_z.publish(pos_camera[2])
            pub_roll.publish(roll_camera)
            pub_pitch.publish(pitch_camera)
            pub_yaw.publish(yaw_camera)

        cv2.namedWindow("Color")
        cv2.imshow("Color", color_image)
        cv2.waitKey(10)

        # for taking pictures
        # k = cv2.waitKey(10)
        # if k % 256 == 32:
        #     img_name = "opencv_frame_{}.png".format(imag_counter)
        #     cv2.imwrite(img_name, color_image)
        #     imag_counter = imag_counter + 1
        
    except CvBridgeError as e:
        print(e)

def tello_detect():
    rospy.init_node("tello_detect", anonymous=True)
    rospy.Subscriber("/tello/raw_image", Image, callback=convert_color_image, queue_size=10)
    # rospy.Subscriber("/tello/image_raw/h264", Image, callback=convert_color_image, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        tello_detect()
    except rospy.ROSInterruptException:
        pass