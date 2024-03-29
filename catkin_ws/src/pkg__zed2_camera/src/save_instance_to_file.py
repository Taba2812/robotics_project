#!usr/bin/env python

import rospy
from std_msgs.msg import String

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import pyzed.sl as sl
import math
import numpy as np
import sys
import cv2 
import struct

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.camera_resolution = sl.RESOLUTION.HD720

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD  # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    # Capture 150 images and depth, then stop
    i = 0
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    tr_np = mirror_ref.m

    while i < 1:
        # A new image is available if grab() returns SUCCESS
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # Retrieve colored point cloud. Point cloud is aligned on the left image.
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            # Get and print distance value in mm at the center of the image
            # We measure the distance camera - object using Euclidean distance
            x = round(image.get_width() / 2)
            y = round(image.get_height() / 2)
            err, point_cloud_value = point_cloud.get_value(x, y)

            distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])

            point_cloud_np = point_cloud.get_data()
            point_cloud_np.dot(tr_np)
            i += 1

            

    # Close the camera
    zed.close()

    cv2.imwrite("./Example_Image_Color_MultipleBlock.png", image.get_data())

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              #PointField('rgb', 12, PointField.UINT32, 1),
              #PointField('rgba', 12, PointField.UINT32, 1),
              ]

    #print (points)

    #print(point_cloud_np)

    points = []

    print("Height")
    print(image.get_height())
    print("Width")
    print(image.get_width())

    for i in range(image.get_height()):
        for j in range(image.get_width()):
            point_cloud_v = point_cloud_np[i,j]
            #print(point_cloud_v)
            x = point_cloud_v[0]
            y = point_cloud_v[1]
            z = point_cloud_v[2]

            pt = [x, y, z]
            points.append(pt)


    print("points transferred")
    header = Header()
    header.frame_id = "map"
    to_return = point_cloud2.create_cloud(header, fields, points)
    #to_return.height = image.get_height()
    #to_return.width = image.get_width()
    return to_return


def ficticious_pc2():
    points = []
    lim = 8
    for i in range(lim):
        for j in range(lim):
            for k in range(lim):
                x = float(i + 1) / lim
                y = float(j + 1) / lim
                z = float(k + 1) / lim

                pt = [x, y, z]

                print("x: ", x, " y: ", y, " z: ", z)
                points.append(pt)

    print("generated ", len(points), " points")

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              #PointField('rgb', 12, PointField.UINT32, 1),
              #PointField('rgba', 12, PointField.UINT32, 1),
              ]

    #print (points)

    header = Header()
    header.frame_id = "map"

    return point_cloud2.create_cloud(header, fields, points)


def talker():
    #pc2 = ficticious_pc2()
    pc2 = main()

    pub = rospy.Publisher('chatter', PointCloud2, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #rospy.loginfo(pc2)
        pub.publish(pc2)
        rate.sleep()

if __name__ == '__main__':
    try:
        #main()
        talker()
    except rospy.ROSInterruptException:
        pass
