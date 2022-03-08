#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import sys

from collections import deque
from imutils.video import VideoStream
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import gc
import argparse
import time
import imutils


def camdet():
    bridge = CvBridge()
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="Path to the (optional) video file")
    ap.add_argument("-b", "--buffer", default=64, type=int, help="max buffer size")
    args = vars(ap.parse_args())

    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)
    pts = deque(maxlen=args["buffer"])

    if not args.get("video", False):
        vs = VideoStream(src=0).start()
    else:
        vs = cv2.VideoCapture(args["video"])

    time.sleep(2.0)


    while True:
        frame = vs.read()
        frame = frame[1] if args.get("video", False) else frame
        if frame is None:
            break

        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        v = 0
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            
            
            M = cv2.moments(c)
           
            center = (int(M['m10']/M['m00']), int(M['m01']/M['m00']))

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                v += 1
        pts.append(center)
        
              
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "marked"
        marker.id = 0
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        points = [0,0]
        for i in  range(v):
            p = Point()
            p.x = int(M['m10']/M['m00'])
            p.y = int(M['m01']/M['m00'])
            
            points.clear()
            points.append(p)
            marker.points = points
            print(marker.points)
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.color.g = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.5
        marker.color.b = 1.0
        marker.lifetime = rospy.Duration()

        
   
              
   
    
        
        


        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        cv2.imshow("Frame", frame)
        rospy.init_node("object_detection",anonymous=True)
        img_pub = rospy.Publisher('webcam/image_raw', Image, queue_size=10)
        rate = rospy.Rate(10)
        img_pub.publish(msg)
        marker_pub = rospy.Publisher("/marker", Marker, queue_size = 100)
        
        marker_pub.publish(marker)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

    if not args.get("video", False):
        vs.stop()
    else:
        vs.release()

    cv2.destroyAllWindows()
    
 
    
 
if __name__ == '__main__':
    try:
        camdet()
    except rospy.ROSInterruptException:
        pass
        
    