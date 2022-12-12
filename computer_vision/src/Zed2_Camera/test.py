import pyzed.sl as sl
import math
import numpy as np
import sys
import cv2 

def main():
    # Create a Camera object
    zed = sl.Camera()



    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.sdk_verbose = False
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD1080 video mode
    init_params.camera_fps = 30  # Set fps at 30
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use PERFORMANCE depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Use meter units (for depth measurements)
    init_params.depth_minimum_distance = 0.5
    init_params.depth_maximum_distance = 30

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print('Camera not Found');
        exit(1)
    
    zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 6)

    # Create and set RuntimeParameters after opening the camera
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL # Use STANDARD sensing mode
    # Setting the depth confidence parameters
    runtime_parameters.confidence_threshold = 30
    runtime_parameters.textureness_confidence_threshold = 100
    
    # A new image is available if grab() returns SUCCESS
    tracking_parameters = sl.PositionalTrackingParameters()
    tracking_parameters.set_as_static=True
    err = zed.enable_positional_tracking(tracking_parameters)

    # Get camera information (ZED serial number)
    zed_serial = zed.get_camera_information().serial_number
    print("Hello! This is my serial number: {0}".format(zed_serial))

    # Capture 150 images and depth, then stop
    i = 0
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    mirror_ref = sl.Transform()
    mirror_ref.set_translation(sl.Translation(2.75,4.0,0))
    tr_np = mirror_ref.m

    while i < 1:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            # Retrieve depth map. Depth is aligned on the left image
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
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

            if not np.isnan(distance) and not np.isinf(distance):
                print("Distance to Camera at ({}, {}) (image center): {:1.3} m".format(x, y, distance), end="\r")
                # Increment the loop
                i = i + 1
            else:
                print("Can't estimate distance at this position.")
                print("Your camera is probably too close to the scene, please move it backwards.\n")
                i = i + 1
            sys.stdout.flush()


    path = "./Image3.png"
    path2 = "./Depth3.jpg"    
    img = image.get_data()
    dp = depth.get_data()
    cv2.imwrite(path, img)
    cv2.imwrite(path2, dp)
    np.save("point_cloud_array3", point_cloud.get_data())
    print(dp)
    cv2.imshow("img",dp)
    cv2.waitKey(0)
    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()