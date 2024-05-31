#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class ImageSizeSubscriber:
    def __init__(self):
        rospy.init_node('temp_kinfu', anonymous=True)
        rospy.Subscriber("/image_size_width", Int32, self.image_width_callback)
        rospy.Subscriber("/image_size_height", Int32, self.image_height_callback)
        rospy.Subscriber("/x_position", Int32, self.x_position_callback)
        rospy.Subscriber("/y_position", Int32, self.y_position_callback)
        self.image_publisher = rospy.Publisher("/image_topic", Image,queue_size=10)
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(1)
        self.image = cv2.imread('/home/yuzhen/catkin_ws_click_interface/src/user_click_pkg/src/dog.jpg')
        self.x_value = None
        self.y_value = None 


    def image_width_callback(self, msg):
        rospy.loginfo("Received image width: %d", msg.data)
        self.y_value = msg.data

    def image_height_callback(self, msg):
        rospy.loginfo("Received image height: %d", msg.data)

    def x_position_callback(self, msg):
        rospy.loginfo("x_position: %d", msg.data)
        self.x_value = msg.data

    def y_position_callback (self, msg):
        rospy.loginfo("y_position: %d", msg.data)
        self.y_value = msg.data

    # def publish_the_img(self,image_path):
    #     image= cv2.imread(image_path)
    #     if image is None:
    #         rospy.logerr("Failed to load image from file '%s'", image_path)
    #         return
    #     ros_image = self.bridge.cv2_to_imgmsg(image,encoding="bgr8")
    #     self.image_publisher.publish(ros_image)
    #     rospy.loginfo("the image has been published")


    def start(self):
        rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
            # rospy.loginfo('publishing image')
            #br = CvBridge()
            if self.image is not None:
                self.image_publisher.publish(self.br.cv2_to_imgmsg(self.image))
                # rospy.loginfo("image has been published")
            self.loop_rate.sleep()
            if self.x_value != None and self.y_value != None:
                rospy.signal_shutdown("close the node")

def main():
    iss = ImageSizeSubscriber()
    # image_path = '/home/yuzhen/catkin_ws_click_interface/src/user_click_pkg/src/dog.jpg'
    iss.start()
    # rospy.spin()

if __name__ == '__main__':
        main()
