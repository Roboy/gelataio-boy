#!/usr/bin/env python

from roboy_cognition_msgs.srv import OrderIceCream
import rospy
import argparse
import sys
import random

def request_icecream(picked_flavour, picked_scoops, service_name):

    print("requesting ", picked_scoops, "scoops of ", picked_flavour, " icecream")
    rospy.wait_for_service("icecream_order")
    service = rospy.ServiceProxy("icecream_order", OrderIceCream)
    responce = service(picked_flavour, picked_scoops)
    
    if responce.success:
        print ("Icecream was successfully requested. ")
    else:
        raise Exception("Icecream request failed: " + responce.error_message)

    rospy.spin()

if __name__=="__main__":
    myargv = rospy.myargv(argv=sys.argv)
    myargv = myargv[1:]

    parser = argparse.ArgumentParser(description = "Sends luigi-style messages to order some icecream from Roboy")
    
    parser.add_argument("-f", "--flavours", type=str, nargs = "+", help = "List of flavours from each to pick random for an order")
    parser.add_argument("-r", "--repeated", type=int, help = "Repeats the msg a given amount of times")
    parser.add_argument("-m", "--min_scoops", type=int, help = "Minimum amount of scoops to order each time")
    parser.add_argument("-M", "--max_scoops", type=int, help = "Maximum amount of scoops to order each time")
    parser.add_argument("-S", "--service_name", type=int, help = "Service name used to order the icecream")

    parsed_vars = parser.parse_args(myargv)
    
   

    rospy.init_node('icecream_requester')
    
    for r in range(parsed_vars.repeated):
        picked_flavour = parsed_vars.flavours[random.randrange(len(parsed_vars.flavours))]
        picked_scoops = random.randrange(parsed_vars.min_scoops, parsed_vars.max_scoops)
        request_icecream(picked_flavour, picked_scoops, parsed_vars.service_name)
    
 

