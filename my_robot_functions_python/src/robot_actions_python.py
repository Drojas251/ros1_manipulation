#! /usr/bin/env python3

import re
from tokenize import group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math

import actionlib
from my_robot_interfaces.msg import MoveRailAction, MoveRailFeedback, MoveRailResult
from my_robot_interfaces.msg import PickPlaceObjAction, PickPlaceObjFeedback, PickPlaceObjResult

import csv

class ManipulationFunctions:
    def __init__(self):
        rospy.init_node('manipulation_functions')

        # Moveit
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.rail = moveit_commander.MoveGroupCommander("rail")
        self.rail.set_max_velocity_scaling_factor(1.0)
        self.rail.set_max_acceleration_scaling_factor(0.2)
        
        self.arm.allow_replanning(True)
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)

        self.ee_link = self.arm.get_end_effector_link()
        self.ee_group_name = "ee"

        # Actions

        self.move_rail_action_ = actionlib.SimpleActionServer(
            'move_rail', 
            MoveRailAction, 
            execute_cb=self.move_rail_, 
            auto_start=True
        )

        self.pick_place_object_action_ = actionlib.SimpleActionServer(
            'pick_place', 
            PickPlaceObjAction, 
            execute_cb=self.pick_place_obj_, 
            auto_start=True
        )

        # Database
        self.obj_database = open('object_database.csv')
        self.obj_location = open('object_location.csv')

        self.obj_database_data = []
        self.obj_location_data = []   

    # ROBOT ROS ACTIONS
    def move_rail_(self,goal):
        # goal is a position value (float)
        _result = MoveRailResult()

        status = self.MoveRail(goal.position)

        if(status):
            self.move_rail_action_.set_succeeded()
            _result.status = True
        else:
            self.move_rail_action_.set_aborted()
            _result.status = False

    def pick_place_obj_(self,goal):
        # goal is the name of object to pick up and name of location to place
        _result = PickPlaceObjResult()

        status = self.PickPlaceObj(goal.object_name, goal.location_name)

        if(status):
            self.pick_place_object_action_.set_succeeded()
            _result.status = True
        else:
            self.pick_place_object_action_.set_aborted()
            _result.status = False

    # ROBOT FUNCTIONS
    def MoveToPose(self,ee_pose):

        status = False

        reach_limit = 0.7

        rail_base_position = self.rail.get_current_joint_values()

        x_diff = ee_pose.position.x - rail_base_position[0]

        if(abs(x_diff) >= reach_limit):
            self.MoveArmToTarget("stow")
            status = self.MoveRail(ee_pose.position.x + 0.1)
            if(status):
                status = self.MoveArmToPose(ee_pose)

        else:
            status = self.MoveArmToPose(ee_pose)

        return status
    
    def MoveArmToPose(self,ee_pose):
        # Moves to a defined Pose
        function_name = "MoveArmToPose"

        current_pose = self.arm.get_current_pose()

        #ee_pose.orientation = current_pose.pose.orientation # remove this later when we solve for orientaiton

        self.arm.set_pose_target(ee_pose)

        #Plan and Execute
        status = self._plan_and_execute(self.arm, function_name)
        
        return status

    def MoveRail(self,position):
        # Moves Raile to a defined Position
        function_name = "MoveRail"

        # Set 
        rail_position = self.rail.get_current_joint_values()
        rail_position[0] = position
        self.rail.set_joint_value_target(rail_position)	

        #Plan and Execute
        status = self._plan_and_execute(self.rail, function_name)
        
        return status   

    def MoveArmToTarget(self,name):
        # moves to stored target
        function_name = "MoveArmToTarget"

        #Set Target
        self.arm.set_named_target(name)

        #Plan and Execute
        status = self._plan_and_execute(self.arm, function_name)
        
        return status

    def MoveRailToTarget(self,name):
        # moves to stored target

        function_name = "MoveRailToTarget"

        #Set Target
        self.rail.set_named_target(name)

        #Plan and Execute
        status = self._plan_and_execute(self.rail, function_name)
        
        return status

    def PickPlaceObj(self,obj_name,location_name):

        obj_pos = self.get_obj_pose(obj_name)
        obj_hieght = self.get_obj_hieght(obj_name)

        obj_pos.orientation.x = 0.5
        obj_pos.orientation.y = 0.5
        obj_pos.orientation.z = -0.5
        obj_pos.orientation.w = 0.5

        #pre-grasp
        obj_pos.position.z = obj_pos.position.z + 2*obj_hieght
        print(obj_name + ": PRE-GRASP")
        self.MoveToPose(obj_pos)

        # grasp
        obj_pos.position.z = obj_pos.position.z - obj_hieght
        print(obj_name + ": GRASP")
        self.MoveToPose(obj_pos)
        self.grasp_obj(obj_name)

        # remove
        obj_pos.position.z = obj_pos.position.z + obj_hieght
        print(obj_name + ": PICK-UP")
        self.MoveToPose(obj_pos)

        place_pose = self.find_pose_from_databae(location_name)
        place_pose.orientation = obj_pos.orientation


        # pre-place
        place_pose.position.z = obj_pos.position.z 
        print(obj_name + ": PRE-PLACE")
        self.MoveToPose(place_pose)

        # place
        place_pose.position.z = obj_pos.position.z - obj_hieght 
        print(obj_name + ": PLACE")
        status = self.MoveToPose(place_pose)
        if(status):
            self.release_obj(obj_name)
            self.update_obj_database(obj_name, location_name)

        # re-treat
        place_pose.position.z = obj_pos.position.z + obj_hieght
        print(obj_name + ": RETREAT")
        self.MoveToPose(place_pose)

        self.MoveArmToTarget("stow")

    def grasp_obj(self,obj_name):
        touch_links = self.robot.get_link_names(group=self.ee_group_name)
        self.scene.attach_box(self.ee_link, obj_name, touch_links=touch_links)

    def release_obj(self,obj_name):
        self.scene.remove_attached_object(self.ee_link, name=obj_name)

    # PLANNING AND EXECUTION
    def _plan_and_execute(self,move_group,function_name):

        #Plan
        success , plan = self._plan(move_group, function_name)
        
        # Check Plan
        if success:
            # execute and check status
            success = self._execute(move_group, plan, function_name) 

            if success:
                status = True # Function Succeeded
            else:
                status = False # Function Failed
        else:
            status = False # Function Failed
        print("")
        return status

    def _plan(self,move_group, function_name):
        #Plan
        error_code_val, plan, planning_time, error_code  = move_group.plan()

        # Get status
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if(success):
            print( "   " + move_group.get_name() + ": Planning Function " + function_name + " -> SUCCESS")
        else:
            print( "   " + move_group.get_name() + ": Planning Function " + function_name + " -> FAIL")

        return success , plan

    def _execute(self,move_group,plan,function_name):
        # execute
        error_code_val = move_group.execute(plan)

        # Get status
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if(success):
            print( "   " + move_group.get_name() + ": Executing Function " + function_name + " -> SUCCESS")
        else:
            print( "   " + move_group.get_name() + ": Executing Function " + function_name + " -> FAIL")

        return success 

    # INTERFACE TO DATABASE
    def update_obj_database(self,obj_name,location_name):
        for obj in self.obj_database_data:
            if obj[0] == obj_name:
                 obj[4] = location_name

    def get_obj_pose(self,obj_name):
        obj_pose = geometry_msgs.msg.Pose()
        found_obj = False

        for obj in self.obj_database_data:
            if obj[0] == obj_name:
                 obj_pose = self.find_pose_from_databae(obj[4])

                 # replace with method to get orientation
                 ee_pose = self.arm.get_current_pose()
                 obj_pose.orientation = ee_pose.pose.orientation
                 obj_pose.position.z =  obj_pose.position.z + float(obj[3])/2 

                 found_obj = True
                 break
        if(found_obj):
            return obj_pose
        else:
            print("OBJ NOT FOUND")
            current_pose = self.arm.get_current_pose()
            return current_pose.pose

    def get_obj_hieght(self,obj_name):
        found_obj = False

        for obj in self.obj_database_data:
            if obj[0] == obj_name:
                 obj_hieght = float(obj[3])
                 found_obj = True
                 break
        if(found_obj):
            return obj_hieght
        else:
            print("OBJ NOT FOUND")
            obj_hieght = 0.05
            return obj_hieght

    def find_pose_from_databae(self,location_name):
        position = geometry_msgs.msg.Pose()
        loc_found = False

        for pos in self.obj_location_data:
            if pos[0] == location_name:
                position.position.x = float(pos[1])
                position.position.y = float(pos[2])
                position.position.z = float(pos[3])

                loc_found = True
                break

        if(loc_found == False):
            print("Location Not Found")
        
        return position

    def initialize_objs(self):

        # read csv files
        obj_database = csv.reader(self.obj_database)
        obj_location = csv.reader(self.obj_location)

        header = []
        header = next(obj_database)
        header = next(obj_location)

        # construct data
        for row in obj_database:
                self.obj_database_data.append(row)
                
        for row in obj_location:
                self.obj_location_data.append(row)

        for obj in self.obj_database_data:
            is_known = obj[0] in self.scene.get_known_object_names()
            if(is_known):
                self.scene.remove_world_object(obj[0])
                rospy.sleep(2.0)

            # add object
            self.add_obj(obj[0])
            rospy.sleep(2.0)

    def clear_scene(self):
        for obj in self.obj_database_data:
            self.scene.remove_world_object(obj[0])
            rospy.sleep(2.0)

    def add_obj(self,name):
        object_name = name
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0

        name_found = False
        position_found = False

        for obj in self.obj_database_data:
            if(obj[0] == name):
                name_found = True
                print("Object " + obj[0] + " Found") 

                for obj_loc in self.obj_location_data:
                    if(obj[4] == obj_loc[0]):
                        print("Object position found")
                        position_found = True

                        box_pose.pose.position.x = float(obj_loc[1])
                        box_pose.pose.position.y = float(obj_loc[2])  
                        box_pose.pose.position.z = float(obj_loc[3]) + float(obj[3])/2

                        self.scene.add_box(object_name, box_pose, size=(float(obj[1]), float(obj[2]), float(obj[3])))

                        break


        if(name_found == False):
            print("Object Name Not Found")    

        if(position_found == False):
            print("Object Position Not Found")    


if __name__ == "__main__":
    rospy.loginfo("Grasp Manipulator On ...")
    my_robot = ManipulationFunctions()

    rospy.sleep(5.0)

    my_robot.initialize_objs()
    print("Robot is Ready")

    # rospy.sleep(5.0)

    # my_robot.PickPlaceObj("small","green")

    # rospy.sleep(5.0)

    # my_robot.PickPlaceObj("large",1)



    rospy.spin()


        

        

    