#!/usr/bin/env python

from roboy_cognition_msgs.msg import IceCream
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension

from std_msgs.msg import Float32MultiArray 
import rospy
import argparse
import sys
import random

from noise import pnoise2

def heightmap_generator(x, y, min_height, max_height, seed):
    heightmap = []
    for i in range(x):
        line = [round(pnoise2 (i/x, j/y, octaves = 4, base = seed), 5) for j in range(y)]
        heightmap.append ([min_height + (max_height-min_height) * j for j in line])
    return heightmap




if __name__=="__main__":
    myargv = rospy.myargv(argv=sys.argv)
    myargv = myargv[1:]

    parser = argparse.ArgumentParser(description = "Sends vision messages with icecream information to Roboy")
    
    parser.add_argument("-f", "--flavour", type=str, action = 'store', help = "The flavour of the icecream")
    parser.add_argument("-F", "--frequency", type=float, help = "Frequency in hz of topic updates.")
    parser.add_argument("-x", "--x_resolution", type=int, help = "Vertical resolution of the heigtmap")
    parser.add_argument("-y", "--y_resolution", type=int, help = "Horizontal resolution of the heigtmap")
    parser.add_argument("-s", "--seed_noise", type=int, help = "Random noise seed")
    parser.add_argument("-t", "--topic_name", type=str, help = "Topic to publish the message")
    parser.add_argument("-m", "--min_height", type=float, help = "Minimum z value")
    parser.add_argument("-M", "--max_height", type=float, help = "Maximum z value")
    parser.add_argument("-p", "--pose", type=float, nargs = 7, help = "Pose of the icecream container in x, y, z, qx, qy, qz, qw, where first three variables are coordinates and later three variables are the orientation quarterion")
    parsed_vars = parser.parse_args(myargv)

    print (parsed_vars.x_resolution)
    map_gen =  heightmap_generator(parsed_vars.x_resolution, parsed_vars.y_resolution, parsed_vars.min_height, parsed_vars.max_height, parsed_vars.seed_noise )

    rospy.init_node('icecream_publisher')
    

    while not rospy.is_shutdown():
        pub = rospy.Publisher(parsed_vars.topic_name, IceCream)
        point = Point(parsed_vars.pose[0], parsed_vars.pose[1], parsed_vars.pose[2])
        quart = Quaternion(parsed_vars.pose[3], parsed_vars.pose[4], parsed_vars.pose[5], parsed_vars.pose[6])
        
        
        
        dim0 = MultiArrayDimension()
        dim1 = MultiArrayDimension()

        dim0.label = "height"
        dim0.size = parsed_vars.x_resolution
        dim0.stride = parsed_vars.x_resolution * parsed_vars.y_resolution
        
        dim1.label = "width"
        dim1.size = parsed_vars.y_resolution
        dim1.stride = parsed_vars.y_resolution

        layout = MultiArrayLayout([dim0, dim1], 0)

        
        ma = Float32MultiArray(layout, [y for x in map_gen for y in x])
        pose = Pose(point, quart)
        
        msg = IceCream(parsed_vars.flavour, pose, ma)
        
        pub.publish(msg)
        rospy.Rate(parsed_vars.frequency).sleep()