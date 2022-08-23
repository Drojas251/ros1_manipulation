#! /usr/bin/env python3
 
import rospy 
import actionlib 
from my_robot_interfaces.msg import PickPlaceObjAction,PickPlaceObjGoal, PickPlaceObjFeedback, PickPlaceObjResult
import sys
from colorama import Fore

def pick_place(object, location):
     # Creates the SimpleActionClient, passing the type of the action
     client = actionlib.SimpleActionClient('pick_place', PickPlaceObjAction)
 
     # Waits until the action server has started up and started
     client.wait_for_server()
 
     # Creates a goal to send to the action server.
     goal = PickPlaceObjGoal(object_name=object, location_name=location)
 
     # Sends the goal to the action server.
     client.send_goal(goal)
 
     # Waits for the server to finish performing the action.
     client.wait_for_result()
     
     return client.get_result()
 
if __name__ == '__main__':
    args = sys.argv[1:]

    # Keyword args:
    #   object: object name to pick up (large, medium, small)
    #   location: location name to place (red, blue, green, yellow, purple)
    #
    # Ex: rosrun my_robot_functions_python pick_place_client.py object="small" location="red"
    
    # Process keyword arguments
    kwargs = {}
    for arg in args:
        key, value = arg.split('=')
        kwargs[key] = value

    object_name = kwargs["object"]
    location_name = kwargs["location"]
    print("")
    print(Fore.YELLOW + f"Goal: place the {object_name} object on the {location_name} platform" + Fore.WHITE)

    try:
        rospy.init_node('pick_place_client')
        result = pick_place(object_name,location_name)
        print(Fore.GREEN + f"Result: Successfully placed the {object_name} object on the {location_name} platform\n" + Fore.WHITE)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")