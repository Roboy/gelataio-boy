#! /usr/bin/env python

# ROS
import rospy, time
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

# Statistics
import numpy as np
from sklearn.decomposition import PCA
from scipy.spatial.transform import Rotation

# Roboy service
from roboy_cognition_msgs.srv import DetectIceCream, DetectIceCreamResponse
from PointDetector import *

# Required for development
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 
from matplotlib import cm
from scipy import signal

# Tmp memory
zed_cam_data = {}
bridge = CvBridge()

def findAngleBetweenVectors(vec_1, vec_2):
    """
    Find angle (rad) between vec_1 and vec_2
    """
    angle = np.arccos(np.clip(np.dot(vec_1, vec_2), -1., 1.))
    return angle / (np.linalg.norm(vec_1) * np.linalg.norm(vec_2))

def rotatePointCloud(point_cloud, angle, axis=np.array([1, 0, 0])):
    """
    Transform from camera to torso frame 
    """
    R = Rotation.from_rotvec(angle * axis).as_dcm()
    return np.dot(point_cloud, R)

def saveSensorDataZEDRGB(data):
    """
    Save incoming RGB data from ZED camera
    """
    global zed_cam_data, bridge
    zed_cam_data['zed_rgb'] = bridge.imgmsg_to_cv2(data, "bgr8")
   
def saveSensorDataRoyalDepth(data):
    """
    Save incoming depth data from royal camera
    """
    
    global zed_cam_data, bridge
    zed_cam_data['royale_depth'] = bridge.imgmsg_to_cv2(data, "32FC1")
    
def saveSensorDataRoyalPC(data):
    """
    Save incoming point cloud from royal camera
    """
    global zed_cam_data
    zed_cam_data['royale_pc'] = data
    
def findScoopingPoint(point_cloud):
    """
    Finds the highest point in the ice cream point cloud.
    """
    # ----- Visualization -----
    fig = plt.figure(figsize=(12,6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(point_cloud[:,0], point_cloud[:,1], point_cloud[:,2], c='blue', alpha=0.1)
    # -------------------------
    
    # Zero mean
    mesh = point_cloud - np.mean(point_cloud, axis=0)

    # PCA
    pca = PCA(n_components=3)
    pca.fit(mesh)
    
    # Rotate around first 2 components
    for i in [0,1]:
        angle = np.pi /2 - findAngleBetweenVectors(pca.components_[i], np.array([0,0,1]))
        rotation_axis = np.cross(pca.components_[i], np.array([0,0,1]))
        mesh = rotatePointCloud(mesh, angle, rotation_axis)

    # Make sure that point is not at the boarder (unreachable)
    X_condition = np.abs(mesh[:,0] - np.mean(mesh[:,0])) < 0.6 * np.abs(np.max(mesh[:,0]) - np.mean(mesh[:,0])) 
    Y_condition = np.abs(mesh[:,1] - np.mean(mesh[:,1])) < 0.6 * np.abs(np.max(mesh[:,1]) - np.mean(mesh[:,1])) 
    # Is on top 20%
    Z_condition = mesh[:,2] - np.mean(mesh[:,2]) > 0.8 * np.abs(np.max(mesh[:,2]) - np.mean(mesh[:,2]))

    in_boundary = np.logical_and(X_condition, Y_condition)
    max_in_boundary = np.logical_and(in_boundary, Z_condition)

    valid_points = point_cloud[max_in_boundary]

    if len(valid_points) == 0:
        # If surface is relatively straight, return a random point within boundaries
        valid_points = point_cloud[in_boundary]

    scoop_point = valid_points[np.random.choice(len(valid_points))]

    # ----- Visualization -----
    ax.scatter(mesh[:,0], mesh[:,1], mesh[:,2], c='green', alpha=0.1)
    ax.scatter(scoop_point[0], scoop_point[1], scoop_point[2], c='red')
    plt.show()
    # -------------------------

    return scoop_point

def getServiceResponse(request):
    """"
    Generate service response

    :param request: Ice cream flavour
    :param type: string

    :return: service reponse
    """
    # Fake class call
    #mesh = np.load(os.path.join(os.path.dirname(__file__), 'cnt_points.npy'))
    #mesh = mesh[np.linspace(0,10000,700).astype('int')]
    #mesh[:,2] += np.cos((mesh[:,0] ** 2 + mesh[:,1] ** 2) * 1000) / 25

    global zed_cam_data
    zed_cam_data['flavor'] = request.flavor
    
    mesh = None
    step_counter = 0

    # Repeat till mesh is found or step counter is too high
    while(mesh is None and step_counter < 50):
        # TODO: average mesh over X amount of steps
        step_counter += 1

        try: 
            print(zed_cam_data.keys())
            mesh = PointDetector.detect(**zed_cam_data)
            mesh = mesh[np.linspace(0,len(mesh)-1,700).astype('int')]
        except TypeError as e:
            # Not enough data in zed_cam_data ... try again
            print("Waiting for camera data...")
            print(e)
            time.sleep(1)
    
    if mesh is None:
        return DetectIceCreamResponse(Point(), Point(), 'Point cloud could not be detected')
    
    # Find a scooping point
    scoop_point = findScoopingPoint(mesh)

    # Prepare response
    start_point = Point()
    start_point.x = scoop_point[0]
    start_point.y = scoop_point[1]
    start_point.z = scoop_point[2]

    return DetectIceCreamResponse(start_point, Point(), '')

if __name__ == '__main__' :
    rospy.init_node('iceCreamMesh', anonymous=True)

    # --- Init service ---
    rospy.Service('iceCreamMeshService', DetectIceCream, getServiceResponse)

    # --- Init subscribers ---
    rospy.Subscriber("/zed/zed_node/left_raw/image_raw_color", Image, saveSensorDataZEDRGB)
    rospy.Subscriber("/royale_camera_driver/depth_image", Image, saveSensorDataRoyalDepth)
    rospy.Subscriber("/royale_camera_driver/point_cloud", PointCloud2, saveSensorDataRoyalPC)

    rospy.spin()