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

import csv

class ManipulationFunctions:
    def __init__(self):
        rospy.init_node('manipulation_functions')
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.rail = moveit_commander.MoveGroupCommander("rail")
        self.rail.set_max_velocity_scaling_factor(1.0)
        self.rail.set_max_acceleration_scaling_factor(1.0)
        
        self.arm.allow_replanning(True)
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)

        self.ee_link = self.arm.get_end_effector_link()
        self.ee_group_name = "ee"

        self._result = MoveRailResult()

        self.move_rail_action_ = actionlib.SimpleActionServer(
            'move_rail', 
            MoveRailAction, 
            execute_cb=self.move_rail_, 
            auto_start=True
        )

        self.obj_database = open('object_database.csv')
        self.obj_location = open('object_location.csv')

        self.obj_database_data = []
        self.obj_location_data = []        

    def move_rail_(self,goal):
        # goal is a position value (float)

        status = self.MoveRail(goal.position)

        if(status):
            self.move_rail_action_.set_succeeded()
            self._result.status = True
        else:
            self.move_rail_action_.set_aborted()
            self._result.status = False

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

        return
    
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

    def PickPlaceObj(self,obj_name,index):
        #obj_pos = geometry_msgs.msg.Pose()
        ee_pose = geometry_msgs.msg.Pose()
        obj_pos = self.get_obj_pose(obj_name)
        obj_hieght = self.get_obj_hieght(obj_name)

        obj_pos.orientation.x = 0.5
        obj_pos.orientation.y = 0.5
        obj_pos.orientation.z = -0.5
        obj_pos.orientation.w = 0.5


        print("Obj hieght", obj_pos.position.z)

        #pre-grasp
        obj_pos.position.z = obj_pos.position.z + 2*obj_hieght + 0.03
        print("PRE-GRASP hieght", obj_pos.position.z)
        self.MoveToPose(obj_pos)

        # grasp
        obj_pos.position.z = obj_pos.position.z - obj_hieght
        print("GRASP hieght", obj_pos.position.z)
        self.MoveToPose(obj_pos)
        self.grasp_obj(obj_name)

        # remove
        obj_pos.position.z = obj_pos.position.z + obj_hieght
        print("REMOVE hieght", obj_pos.position.z)
        self.MoveToPose(obj_pos)

        place_pose = self.find_pose_from_databae(index)
        place_pose.orientation = obj_pos.orientation
        ee_pose = place_pose
        print(" INITIAL PLACE hieght", ee_pose.position.z)

        # pre-place
        ee_pose.position.z = obj_pos.position.z + obj_hieght + 0.05
        print("PRE-PLACE hieght", ee_pose.position.z)
        self.MoveToPose(ee_pose)

        # place
        ee_pose.position.z = obj_pos.position.z + obj_hieght 
        print("PLACE hieght", ee_pose.position.z)
        self.MoveToPose(ee_pose)
        self.release_obj(obj_name)

        # re-treat
        ee_pose.position.z = obj_pos.position.z + obj_hieght + 0.05
        print("RETREAT hieght", ee_pose.position.z)
        self.MoveToPose(ee_pose)

    def grasp_obj(self,obj_name):
        touch_links = self.robot.get_link_names(group=self.ee_group_name)
        self.scene.attach_box(self.ee_link, obj_name, touch_links=touch_links)

    def release_obj(self,obj_name):
        self.scene.remove_attached_object(self.ee_link, name=obj_name)

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

    def get_obj_pose(self,obj_name):
        obj_pose = geometry_msgs.msg.Pose()
        found_obj = False

        for obj in self.obj_database_data:
            if obj[0] == obj_name:
                 obj_pose = self.find_pose_from_databae(float(obj[5]))

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

    def find_pose_from_databae(self,index):
        position = geometry_msgs.msg.Pose()
        loc_found = False

        for pos in self.obj_location_data:
            if float(pos[0]) == float(index):
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
                    if(obj[5] == obj_loc[0]):
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



    # def GetGrasps(self,object_id):
    #     req = GraspsRequest()
    #     req.object_id = object_id
    #     print('waiting grasp planner')
    #     rospy.wait_for_service('pick_grasps_service')
    #     print('done')
    #     resp = self.get_pick_grasps(req)
    #     resp.grasps.sort(key=lambda x: x.grasp_quality, reverse=True)

    #     return resp

    


if __name__ == "__main__":
    rospy.loginfo("Grasp Manipulator On ...")
    my_robot = ManipulationFunctions()

    my_robot.initialize_objs()

    rospy.sleep(5.0)

    my_robot.PickPlaceObj("blue_cube",5)

    rospy.sleep(5.0)

    my_robot.PickPlaceObj("green_cube",1)

    print("DONE")





    # my_robot.MoveArmToTarget("ready")
    # # my_robot.MoveRailToTarget("right")
    # my_robot.MoveRail(0.1)

    # my_robot.MoveRail(1.2)


    # pose_target = geometry_msgs.msg.Pose()
    # pose_target.orientation.w = 1.0
    # pose_target.position.x = 1.2
    # pose_target.position.y = -0.7
    # pose_target.position.z = 0.45

    # my_robot.MoveToPose(pose_target)


    rospy.spin()


        

        

    