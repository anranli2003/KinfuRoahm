#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32 
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 
import cv2

class ImageSizePublisher:
    def __init__(self):
        rospy.init_node('user_interface', anonymous=True)
        self.size_pub_width = rospy.Publisher("/image_size_width", Int32, queue_size=10)
        self.size_pub_height = rospy.Publisher("/image_size_height", Int32, queue_size=10)
        # self.image_path = "dog.jpg"  # Change this to your image filename
        self.clicked_points = []

        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber ("/image_topics",Image, self.image_callback)
        self.cv_image_subscribed = None
        self.received_img = False
        self.rate = rospy.Rate(1)
        self.pub_x = rospy.Publisher("/x_position", Int32, queue_size=10)
        self.pub_y = rospy.Publisher("/y_position", Int32, queue_size=10)

    #when i use the outside direction:
        
    # def load_and_publish_image_size(self, image_path):
    #     image = cv2.imread(image_path)
    #     if image is None:
    #         rospy.logerr("Failed to load image from file '%s'", image_path)
    #         return
    #     height, width, _ = image.shape
    #     self.size_pub_width.publish(width)
    #     self.size_pub_height.publish(height)
        
    def load_and_publish_image_size(self):
        image = self.cv_image_subscribed

        height, width, _ = image.shape
        self.size_pub_width.publish(width)
        self.size_pub_height.publish(height)

    # def display_image(self):
    #     cv_image = self.cv_image_subscribed
    #     cv2.imshow("Image Window", cv_image)
    #     cv2.setMouseCallback("Image Window", self.mouse_callback)
    #     while True:
    #         key = cv2.waitKey(1) & 0xFF
    #         if key == ord('q'):
    #             break
    #     cv2.destroyAllWindows()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_points.append((x, y))
            print("Clicked at (x={}, y={})".format(x, y))
            self.pub_x.publish(x)
            self.pub_y.publish(y)
            rospy.signal_shutdown("close the node")


    def image_callback(self, msg):
        self.cv_image_subscribed = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #testing
        cv_image = self.cv_image_subscribed
        cv2.imshow("Image Window", cv_image)
        cv2.setMouseCallback("Image Window", self.mouse_callback)
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        cv2.destroyAllWindows()
        self.received_img = True


    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Waiting for message...")
            self.rate.sleep()

def main():
    isp = ImageSizePublisher()
    # image_path = 'dog.jpg'
    # isp.load_and_publish_image_size(image_path)

    # while isp.received_img == False:
    #     print("wait\n")
    #     if isp.received_img == False:





    # isp.display_image()
    rospy.spin()

if __name__ == '__main__':
    main()
