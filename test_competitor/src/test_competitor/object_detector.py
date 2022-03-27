#!/usr/bin/env python

import rospy
import cv2
import math
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from sensor_msgs.msg import Image
from test_competitor.msg import Part
from test_competitor.srv._DetectObjects import DetectObjects, DetectObjectsResponse, DetectObjectsRequest
from cv_bridge import CvBridge, CvBridgeError 

class ObjectDetector:
    colors = [(0,0,255), (0,255,0), (255,0,0)]
    tray_length = 1.76/2
    tray_width = 1.54/2
    bin_length = 0.6
    bin_width = 0.6
    
    def __init__(self):    
        # Check to make sure camera topics exists
        self.gantry_tray_camera_topic = '/ariac/gantry/gantry_tray_camera/gantry_tray_camera/color/image_raw'
        if not self.topic_exists(self.gantry_tray_camera_topic):
            exit()
            
        self.gantry_bin_camera_topic = '/ariac/gantry/gantry_bin_camera/gantry_bin_camera/color/image_raw'
        if not self.topic_exists(self.gantry_bin_camera_topic):
            exit()
        
        # Create cv bridge
        self.bridge = CvBridge()
        
        # Create TF Listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # Subscribe to image topic
        self.gantry_tray_raw_image = None
        self.gantry_bin_raw_image = None
        self.gantry_tray_camera_sub = rospy.Subscriber(self.gantry_tray_camera_topic, Image, self.gantry_tray_image_callback)
        self.gantry_bin_camera_sub = rospy.Subscriber(self.gantry_bin_camera_topic, Image, self.gantry_bin_image_callback)
        
        # Advertise service
        self.list_parts = rospy.Service('list_parts', DetectObjects, self.detect_objects)
        
    def detect_objects(self, request):
        response = DetectObjectsResponse()
                 
        if request.location == DetectObjectsRequest.TRAY:
            image = self.transform_tray_image()
        else:
            if self.gantry_over_bin(request.location):
                image = self.transform_bin_image()
            else:
                rospy.logerr("Unable to detect parts, gantry is not in correct position")
                response.success = False
                return response
        
        contours = self.find_contours(image)
        
        for contour in contours:
            part = Part()
            part.pose = self.determine_part_pose(contour, image, request.location)
            part.color = self.determine_part_color(contour, image)
            part.type = self.determine_part_type(contour, part.pose.pose.position)
            response.parts.append(part)
            
            cv2.drawContours(image, [contour], 0, self.colors[part.color], 2)
        
        response.image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        response.success = True
        
        return response
            
    def transform_tray_image(self):
        frame = self.gantry_tray_raw_image
        min_x, min_y, max_x, max_y = 159, 7, 688, 466
        corners = np.float32([[min_x, max_y], [min_x, min_y], [max_x, min_y], [max_x, max_y]])

        im_height = 500
        im_width = int(im_height*(self.tray_length/self.tray_width))

        dst = np.float32([[0, im_height], [0, 0], [im_width, 0], [im_width, im_height]])
        M = cv2.getPerspectiveTransform(corners, dst)

        overhead = cv2.warpPerspective(frame, M, (im_width, im_height))
        
        return overhead

    def transform_bin_image(self):
        frame = self.gantry_bin_raw_image
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	    
        _, threshold = cv2.threshold(gray, 215, 255, cv2.THRESH_BINARY)
            
        _, contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        areas = []
        for cnt in contours:
            areas.append(cv2.contourArea(cnt))

        bin_cnt = contours[areas.index(max(areas))]

        rect = cv2.minAreaRect(bin_cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        corners = self.order_points(box)

        im_width, im_height = 500, 500
        dst = np.float32([[0, im_height], [0, 0], [im_width, 0], [im_width, im_height]])
        M = cv2.getPerspectiveTransform(corners, dst)

        overhead = cv2.warpPerspective(frame, M, (im_width, im_height))
            
        return overhead
        
    def find_contours(self, image, ):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        _, threshold = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY)
        
        _, contours, tree = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Determine which contours are parts
        objects = []
        for i in range(len(contours)):
            parent = tree[0][i][3]
            area = cv2.contourArea(contours[i])
            if parent == 0 and area > 1000:
                objects.append(contours[i])
        
        return objects

    def determine_part_pose(self, contour, image, location):
        # determine location of centroid on image
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        dx = float(cx - image.shape[1]/2)
        dy = -float(cy - image.shape[0]/2)
        
        if location == DetectObjectsRequest.TRAY:
            x_pos = self.tray_length * (dx/image.shape[1])
            y_pos = self.tray_width * (dy/image.shape[0])
        else:
            x_pos = self.bin_length * (dx/image.shape[1])
            y_pos = self.bin_width * (dy/image.shape[0])
        
        stamped_pose = PoseStamped()
        if location == DetectObjectsRequest.TRAY:
            stamped_pose.header.frame_id = 'torso_tray'
        else:
            stamped_pose.header.frame_id = 'bin' + str(location) + '_frame'
            
        stamped_pose.pose.position = Vector3(x_pos, y_pos, 0)
        stamped_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        
        return stamped_pose
    
    def determine_part_color(self, contour, image):
        mask = np.zeros(image.shape[:2], np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)

        mean = cv2.mean(image, mask)[:3]
    
        if max(mean) == mean[0]:
            color = Part.BLUE
        elif max(mean) == mean[1]:
            color = Part.GREEN
        else:
            color = Part.RED
        
        return color
    
    def determine_part_type(self, contour, position):
        _,_,w,h = cv2.boundingRect(contour)
        
        area = cv2.contourArea(contour)
        
        # Check aspect ratio for battery
        aspect_ratio = h/float(w)
        if w > h:
            aspect_ratio = w/float(h)
        
        if aspect_ratio > 1.5:
            return Part.BATTERY
        else:
            # Part polynomial constants 
            coeffs = {Part.REGULATOR: [4.4, 0.3, 17.6],
                    Part.PUMP: [10, -0.4, 70],
                    Part.SENSOR: [4.7, 0.9, 20.7]}
            
            r = math.sqrt(position.x**2 + position.y**2)
            
            diffs = {}
            for name in coeffs.keys():
                a, b, c = coeffs[name]
                diff = abs(area/1000.0 - (a + b*r + c*(r**2)))
                diffs[name] = diff
            
        return min(diffs, key=diffs.get)                 
    
    def gantry_tray_image_callback(self, msg):
        try:
            self.gantry_tray_raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            
    def gantry_bin_image_callback(self, msg):
        try:
            self.gantry_bin_raw_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            
    def gantry_over_bin(self, location):
        bin_frame = 'bin' + str(location) + '_frame'
        gantry_frame = 'torso_base'

        try:
            transform = self.tfBuffer.lookup_transform(gantry_frame, bin_frame, rospy.Time(), rospy.Duration(1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Unable to lookup transform")
            return False

        distance = math.sqrt(transform.translation.x**2 + transform.translation.y**2)

        if distance < 0.2:
            return True
        else:
            print(transform.translation)
            return False
    
    def topic_exists(self, topic_name):
        topic_exists = False
        
        for name, _ in rospy.get_published_topics('/ariac'):
            if name == topic_name:
                topic_exists = True
                break
        
        if topic_exists:
            return True
        else:
            rospy.logerr("Cannot find {name}".format(name=topic_name))
            return False
        
    @staticmethod
    def order_points(pts):
        xSorted = pts[np.argsort(pts[:, 0]), :]

        leftMost = xSorted[:2, :]
        rightMost = xSorted[2:, :]

        leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
        (tl, bl) = leftMost

        D = []
        for pt in rightMost:
            D.append(math.sqrt((tl[0]-pt[0])**2 + (tl[1]-pt[1])**2))

        (br, tr) = rightMost[np.argsort(D)[::-1], :]

        return np.array([bl, tl, tr, br], dtype="float32")
    