#!/usr/bin/env python3
import rospy
import numpy as np
import time
import sys

from std_msgs.msg import String
from asl_turtlebot.msg import DetectedObjectList

class RescueList:
    """
    This node handles taking the list of animals that the user wants to rescue.
    It is the sole node that should publish to animals_to_rescue.
    """

    def __init__(self):
        rospy.init_node("animal_rescuer", anonymous=True)
        self.resue_list = rospy.Publisher("/list_to_rescue", DetectedObjectList, queue_size=10)

    def send_names(self, list_animals):
        new_obj = DetectedObjectList(list_animals, [])
        self.resue_list.publish(new_obj)
        print("published", new_obj)

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()
 
if __name__ == '__main__':
    # total arguments
    n = len(sys.argv)
    
    print("\nArguments passed:", end = " ")
    for i in range(1, n):
        print(sys.argv[i], end = " ")


    rl = RescueList()
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        rl.send_names(sys.argv[1:])
        rate.sleep()
    
    print("published")
    # rl.run()
