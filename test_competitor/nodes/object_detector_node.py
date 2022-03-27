#!/usr/bin/env python

import rospy
from test_competitor.object_detector import ObjectDetector

def main():
    rospy.init_node("object_detector")
    
    detector = ObjectDetector()
    
    rospy.spin()

if __name__ == "__main__":
    main()
