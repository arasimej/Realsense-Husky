#!/usr/bin/env python3
import cv2
import imutils
import argparse
from realsense_camera import *
from mask_rcnn import *

#Loading the realsense camera
rs = RealsenseCamera()
mrcnn = MaskRCNN()

#SINGLE IMAGE TEST:
# ret, bgr_frame, depth_frame = rs.get_frame_stream()

# cv2.imshow("Bgr frame", bgr_frame)
# cv2.waitKey(0)

# sensorLeft = None
# sensorCenter = None
# sensorRight = None

#MULTIPLE IMAGES TEST:
while True:
    #ret, bgr_frame, depth_frame = rs.get_frame_stream() # ON FOR ALL
    #__________________________
    # Depth Frame Print Values
    # print(depth_frame)
    #__________________________
    # Depth Frame Print Values Cleaned Up Single Stream 
    # distance_mm = depth_frame[188, 250]
    # print(distance_mm)
    #__________________________
    # THE DOT
    # point_x, point_y = 250, 188
    # distance_mm = depth_frame[point_x, point_y]

    # cv2.circle(bgr_frame, (point_x, point_y), 8, (0, 0, 255), -1)

    # cv2.imshow("depth frame", depth_frame)
    # cv2.imshow("Bgr frame", bgr_frame)
    #__________________________
    # Text for Depth with Dot
    # point_x, point_y = 250, 188
    # distance_mm = depth_frame[point_x, point_y]

    # cv2.circle(bgr_frame, (point_x, point_y), 8, (0, 0, 255), -1)
    # cv2.putText(bgr_frame, "{} mm".format(distance_mm), (point_x, point_y- 10), 0, 1, (0, 0, 255), 2)

    # cv2.imshow("depth frame", depth_frame)
    # cv2.imshow("Bgr frame", bgr_frame)
    #__________________________
    # Posting Mask Values
    # boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)
    # print("boxes", boxes)
    # print("classes", classes)
    # print("contours", contours)
    # print("centers", centers)
    #__________________________
    # boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)

    # distance_mm = depth_frame[188, 250]
    # print(distance_mm)
    
    # bgr_frame = mrcnn.draw_object_mask(bgr_frame)
    # mrcnn.draw_object_info(bgr_frame, depth_frame)

    # cv2.imshow("depth frame", depth_frame)
    # cv2.imshow("Bgr frame", bgr_frame)

    #-------------------------------------------------------
    ret, bgr_frame, depth_frame = rs.get_frame_stream()

    # CENTER
    boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)
    if centers is not None: 
        DetectedCenterPrint = [item for item in centers
                               if item[0] <= 400]
    #     print("Center:", DetectedCenterPrint)# Tuples:...[(0,0),...

    bgr_frame = mrcnn.draw_object_mask(bgr_frame)
    mrcnn.draw_object_info(bgr_frame, depth_frame)

    # ITEM NAME PRINT
    for class_id in mrcnn.obj_classes:
        print("Thing:", mrcnn.classes[int(class_id)])
    if mrcnn.classes[int(class_id)] == "bottle":
        print("BOTTLLLE (not chair)")

    # THE DOT
    point_x, point_y = 320, 200
    # Item Depth Detection:
    # distance_mm = depth_frame[point_x, point_y]
    # print("Depth Measure:", distance_mm)

    # IMAGE DISPLAY
    cv2.circle(bgr_frame, (point_x, point_y), 8, (0, 0, 255), -1)
    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Bgr frame", bgr_frame)

    # HUSKY REACTION
    for class_id in mrcnn.obj_classes:
        if centers is not None:     
                if mrcnn.classes[int(class_id)] == "bottle":
                        if centers[0][0] <= 160:
                                print("Object Detected Left") 
                                # readings.linear.x = 0
                                # readings.angular.z = 0.2
                                # pub.publish(readings)
                                print(" ")
                        if (centers[0][0] > 160 and centers[0][0] < 480):
                                print("Object Detected Center")
                        #         # readings.linear.x = 0.2
                        #         # readings.angular.z = 0
                        #         # pub.publish(readings)
                                print(" ")
                        if centers[0][0] >= 480:
                                print("Object Detected Right")
                        #         # readings.linear.x = 0
                        #         # readings.angular.z = -0.2
                        #         # pub.publish(readings)
                                print(" ")
                        else:
                                print("No Bottle Seen")
                                # readings.linear.x = 0
                                # readings.angular.z = 0
                                # pub.publish(readings)
                                print(" ")



    # print("boxes", boxes)
    # print("classes", classes)
    # print("contours", contours)
    # print("centers", centers)

    print(" ")

    #__________________________

    key = cv2.waitKey(1)
    if key == 27:
        break