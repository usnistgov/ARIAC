#!/usr/bin/env python

import rospy
import cv2
import tf2_ros
from test_competitor.srv import MoveGantry, MoveGantryRequest
from test_competitor.srv import DetectObjects, DetectObjectsRequest
from cv_bridge import CvBridge, CvBridgeError


def main():
    rospy.init_node("test_competitor")

    bridge = CvBridge()

    detect_tray_objects = rospy.ServiceProxy("list_parts", DetectObjects)
    move_gantry = rospy.ServiceProxy('move_gantry', MoveGantry)

    request = DetectObjectsRequest()
    request.location = DetectObjectsRequest.BIN3

    # Create TF Listener
    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)

    try:
        bin_transform = tfBuffer.lookup_transform(
            'world', 'bin3_frame', rospy.Time(), rospy.Duration(1)).transform
        gantry_transform = tfBuffer.lookup_transform(
            'world', 'torso_base', rospy.Time(), rospy.Duration(1)).transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr("Unable to lookup transform")
        exit()

    move_gantry_request = MoveGantryRequest()
    move_gantry_request.move_y = gantry_transform.translation.y - \
        bin_transform.translation.y

    response = move_gantry(move_gantry_request)

    move_gantry_request = MoveGantryRequest()
    move_gantry_request.move_x = bin_transform.translation.x - \
        gantry_transform.translation.x

    response = move_gantry(move_gantry_request)

    response = detect_tray_objects(request)
    if response.success:
        try:
            detected_image = bridge.imgmsg_to_cv2(response.image, "bgr8")
        except CvBridgeError as e:
            print(e)
            rospy.logerr("Unable to process image")
            exit()

        print("{num} parts detected\n".format(num=len(response.parts)))

        colors = ['red', 'green', 'blue']
        types = ['pump', 'sensor', 'regulator', 'battery']

        part_num = 0
        for part in response.parts:
            print("Part #{num}".format(num=part_num))
            print("\tposition: ({x}, {y})".format(
                x=part.pose.pose.position.x, y=part.pose.pose.position.y))
            print("\ttype: {type}".format(type=types[part.type]))
            print("\tcolor: {color}".format(color=colors[part.color]))
            part_num += 1

        cv2.imshow('Detected Objects', detected_image)

        cv2.waitKey(0)
    else:
        rospy.logerr("Unable to detect parts")


if __name__ == "__main__":
    main()
