#!usr/bin/env python

import rospy
import cv2
import os

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import Bool, String

src_path = os.getcwd() + "/src/images_database/complete_data_examples/"
point_cloud_file_path = src_path + "Example_Pcl_SingleBlock.png"
image_mat_file_path = src_path + "Example_Image_Color_SingleBlock.png"

def generate_data():
    image = cv2.imread(image_mat_file_path)
    point_cloud = cv2.imread(point_cloud_file_path)

    return point_cloud, image

def elaborate_point_cloud(point_cloud, image):
    
    fields = [PointField('x',  0, PointField.FLOAT32, 1),
              PointField('y',  4, PointField.FLOAT32, 1),
              PointField('z',  8, PointField.FLOAT32, 1),]

    points = []

    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            
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

def fake_data_callback(data):
    data_pub = rospy.Publisher('Camera_Data', PointCloud2, queue_size=10)
    imag_pub = rospy.Publisher('Image_String', String, queue_size=10)

    point_cloud, image = generate_data()
    result = elaborate_point_cloud(point_cloud, image)
    data_pub.publish(result)
    imag_pub.publish(image_mat_file_path)



if __name__ == '__main__':
    
    print(image_mat_file_path)
    #try:
    #    rospy.init_node('Camera_Dummy')
        
    #    sub = rospy.Subscriber('Camera_Request', Bool, callback=fake_data_callback)
    #    rospy.spin()
    #except rospy.ROSInterruptException:
    #    pass