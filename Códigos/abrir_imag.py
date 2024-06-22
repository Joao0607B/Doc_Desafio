#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Converte a mensagem ROS Image para uma imagem OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr("Erro na conversão da imagem: %s", e)

def main():
    rospy.init_node('camera_view_node', anonymous=True)
    
    rospy.Subscriber('/edrn/camera/color/image_raw', Image, image_callback)
    
    rospy.loginfo("Esperando imagens da câmera...")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Encerrando...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
