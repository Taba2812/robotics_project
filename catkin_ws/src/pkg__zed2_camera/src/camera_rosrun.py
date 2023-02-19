#!usr/bin/env python

import rospy

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import Bool

import pyzed.sl as sl
import numpy as np
import cv2 

def get_camera_data():
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
    image = sl.Mat()
    point_cloud = sl.Mat()

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    tr_np = mirror_ref.m

    # Take actual picture
    # A new image is available if grab() returns SUCCESS
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # Retrieve left image
        zed.retrieve_image(image, sl.VIEW.LEFT)
        # Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        point_cloud_np = point_cloud.get_data()
        point_cloud_np.dot(tr_np)

    # Close the camera
    zed.close()

    # Save PNG Image to local storage
    cv2.imwrite("./RGB_Image.png", image.get_data())

    # Return raw Point Cloud
    return point_cloud_np, image;

def elaborate_point_cloud(point_cloud, image):
    
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),]

    points = []

    for i in range(image.get_height()):
        for j in range(image.get_width()):
            
            point_cloud_v = point_cloud[i,j]
            x = point_cloud_v[0]
            y = point_cloud_v[1]
            z = point_cloud_v[2]

            pt = [x, y, z]
            points.append(pt)

    header = Header()
    header.frame_id = "map"
    to_return = point_cloud2.create_cloud(header, fields, points)

    return to_return

def take_data(data):
    if data.data:
        sender = rospy.Publisher('Camera_Pcl', PointCloud2, queue_size=10)
        point_cloud, image = get_camera_data()
        result = elaborate_point_cloud(point_cloud, image)
        sender.publish(result)

def talker():
    rospy.init_node('Zed2_Camera')
    rate = rospy.Rate(10)
    print('initiating')
    rec = rospy.Subscriber('Camera_Bool', Bool, callback=take_data)

    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
