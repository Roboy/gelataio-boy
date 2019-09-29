#! /usr/bin/env python

# ROS
import rospy, time, tf
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Point, PointStamped
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

zed_cam_data = {}
bridge = CvBridge()

pc_publisher = None
point_publisher = None

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
    
def transformCam2Torso(point_cloud):
    """
    Transform from camera to torso frame
    """
    rotAroundX = rotatePointCloud(point_cloud, np.pi/2, np.array([1,0,0]))
    rotAroundXZ = rotatePointCloud(rotAroundX, np.pi, np.array([0,0,1]))
    return rotAroundXZ + np.array([0.01,-0.01, 0.05])

def getPointCloud2Msg(mesh):
    """
    Get PointCloud2 msg from an array
    """
    msg = PointCloud2()
    msg.header.frame_id = 'torso'

    msg.height = 1
    msg.width = len(mesh)
    
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
        ]
    
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * mesh.shape[0]
    msg.is_dense = int(np.isfinite(mesh).all())

    msg.data = np.asarray(mesh, np.float32).tostring()

    return msg

def findScoopingPoint(point_cloud):
    """
    Finds the highest point in the ice cream point cloud.
    """
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

    return scoop_point

def getServiceResponse(request):
    """"
    Generate service response

    :param request: Ice cream flavour
    :param type: string

    :return: service reponse
    """
    # Fake class call
    # Uncomment the line below to start the code on prerecorded data
    #mesh = np.load(os.path.join(os.path.dirname(__file__), 'flakes.npy'))
    
    global zed_cam_data, pc_publisher

    zed_cam_data['flavor'] = request.flavor
    
    mesh = None
    step_counter = 0

    # Repeat till mesh is found or step counter is too high
    while (mesh is None or len(mesh) < 100) and step_counter < 50:
        step_counter += 1

        try: 
            mesh = PointDetector.detect(**zed_cam_data)
            print(mesh)
        except TypeError as e:
            # Not enough data in zed_cam_data ... try again
            print("Waiting for camera data...")
            print(e)
            time.sleep(1)

    if mesh is None or len(mesh) < 100:
        return DetectIceCreamResponse(PointStamped(), PointStamped(), 'Could not detect ice-cream')

    mesh = transformCam2Torso(mesh)

    # ----- RVIZ Visualization -----
    msg = getPointCloud2Msg(mesh)
    pc_publisher.publish(msg)
    # -------------------------

    # Find a scooping point
    scoop_point = findScoopingPoint(mesh)

    point_stamped = PointStamped()
    point_stamped.header.frame_id = "torso"

    # Prepare response
    start_point = Point()
    start_point.x = scoop_point[0]
    start_point.y = scoop_point[1]
    start_point.z = scoop_point[2]

    point_stamped.point = start_point

    point_publisher.publish(point_stamped)

    return DetectIceCreamResponse(point_stamped, PointStamped(), '')

if __name__ == '__main__' :
    rospy.init_node('iceCreamService')

    # --- Init service ---
    rospy.Service('iceCreamService', DetectIceCream, getServiceResponse)

    # --- Init subscribers ---
    rospy.Subscriber("/pico_flexx/image_depth", Image, saveSensorDataRoyalDepth)
    rospy.Subscriber("/pico_flexx/points", PointCloud2, saveSensorDataRoyalPC)

    # Init point cloud (for rviz) publisher
    pc_publisher = rospy.Publisher("/ice_cream_pc", PointCloud2, queue_size=10)
    point_publisher = rospy.Publisher("/ice_cream_scoop_point", PointStamped, queue_size=10)

    rospy.spin()