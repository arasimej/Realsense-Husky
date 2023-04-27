#!/usr/bin/env python3

import rospy
import cv2
import imutils
from realsense_camera import *
from mask_rcnn import *
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

rs = RealsenseCamera()
mrcnn = MaskRCNN()

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
readings = Twist()

def callback(data):
    # REALSENSE OBJECT DETECTION STUFF
    ret, bgr_frame, depth_frame = rs.get_frame_stream()

    boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)
    bgr_frame = mrcnn.draw_object_mask(bgr_frame)
    mrcnn.draw_object_info(bgr_frame, depth_frame)

    # THE DOT
    point_x, point_y = 250, 188
    distance_mm = depth_frame[point_x, point_y]

    cv2.circle(bgr_frame, (point_x, point_y), 8, (0, 0, 255), -1)

    cv2.imshow("depth frame", depth_frame)
    cv2.imshow("Bgr frame", bgr_frame)

def main():
    global pub
    global ret 
    global bgr_frame
    global depth_frame
    global boxes 
    global classes 
    global contours 
    global centers 

    while True:

        ret, bgr_frame, depth_frame = rs.get_frame_stream()

        # CENTER
        boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)
        if centers is not None: 
            DetectedCenterPrint = [item for item in centers
                                if item[0] <= 400]
            #print("Center:", DetectedCenterPrint)# Tuples:...[(0,0),...

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
        if centers is not None:     
            if mrcnn.classes[int(class_id)] == "bottle":
                if centers[0][0] <= 160:
                        print("Object Detected Left") 
                        readings.linear.x = 0
                        readings.angular.z = 0.4
                        pub.publish(readings)
                        print(" ")
                if (centers[0][0] > 160 and centers[0][0] < 480):
                        print("Object Detected Center")
                        readings.linear.x = 0.2
                        readings.angular.z = 0
                        pub.publish(readings)
                        print(" ")
                if centers[0][0] >= 480:
                        print("Object Detected Right")
                        readings.linear.x = 0
                        readings.angular.z = -0.4
                        pub.publish(readings)
                        print(" ")
                else:
                        print("No Bottle Seen")
                        readings.linear.x = 0
                        readings.angular.z = 0
                        pub.publish(readings)
                        print(" ")

        print(" ")

        key = cv2.waitKey(1)
        if key == 27:
            break
                  
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    sub = rospy.Subscriber("/sonar", PointCloud, callback)
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass  



# #!/usr/bin/env python
# import rospy
# from std_msgs.msg import Float32MultiArray
# from nav_msgs.msg import Odometry

# pub = []
# WHEEL_DIA = 0.195
# WHEEL_BASE = 0.331

# def compute_wheel_vel(v,w):
#     w_r = (2*v + w*WHEEL_BASE)/WHEEL_DIA
#     w_l = (2*v - w*WHEEL_BASE)/WHEEL_DIA
#     return w_l, w_r

# def callback(data):
#     global pub
#     v = data.twist.twist.linear.x
#     w = data.twist.twist.angular.z
#     w_left, w_right = compute_wheel_vel(v,w)
#     msg = Float32MultiArray()
#     msg.data = [w_left, w_right]
#     pub.publish(msg)
    
# def wheel_vel():
#     global pub
#     rospy.init_node('listener', anonymous=True)
#     pub = rospy.Publisher('/wheel_velocities', Float32MultiArray, queue_size=10)
#     rospy.Subscriber("/odom", Odometry, callback)
#     rospy.spin()

# if __name__ == '__main__':
#     wheel_vel()