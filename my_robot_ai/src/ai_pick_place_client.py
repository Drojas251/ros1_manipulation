#! /usr/bin/env python3
 
import rospy 
import actionlib 
from my_robot_interfaces.msg import PickPlaceObjAction,PickPlaceObjGoal
from colorama import Fore
from model import ActionChooser

class SmartActionClient:
    def __init__(self):
        self.model = ActionChooser() 

        self.client = actionlib.SimpleActionClient('pick_place', PickPlaceObjAction)
        self.client.wait_for_server()
    
    def wait_for_input(self):
        # Input loop
        run = True
        while run:
            user_input = input("Ask robot to move a block or enter 'exit' to quit: ")

            if user_input == "exit":
                run = False
            else:
                size, color = self.predict_action([user_input])
                self.pick_place(size, color)
            
    def predict_action(self, text):
        return self.model.predict(text)

    def pick_place(self, size, color):
    
        print(Fore.GREEN + f"Goal: place the {size} block on the {color} platform\n" + Fore.WHITE)

        # Creates a goal to send to the action server.
        goal = PickPlaceObjGoal(object_name=size, location_name=color)
    
        # Sends the goal to the action server.
        self.client.send_goal(goal)
    
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        return self.client.get_result()
 
if __name__ == '__main__':

    rospy.init_node('pick_place_client')

    smart_action = SmartActionClient()
    smart_action.wait_for_input()