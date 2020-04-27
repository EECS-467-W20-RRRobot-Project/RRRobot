#!/usr/bin/env python
# image_display.py

import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def callback(data):
    print("IMAGE DISPLAY CALLBACK")
    file = data.data

    img = mpimg.imread(file)
    imgplot = plt.imshow(img)
    plt.show()

def listener():
    rospy.init_node('image_display', anonymous=False)
    rospy.Subscriber("/current_image", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
