#! /usr/bin/env python3

import rospy
import moveit_commander
import sys
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import math



class ManipulationFunctions:
    def __init__(self):
        rospy.init_node('manipulation_functions')
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        self.arm = moveit_commander.MoveGroupCommander("manipulator")
        self.rail = moveit_commander.MoveGroupCommander("rail")
        
        self.arm.allow_replanning(True)
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)

    
    def MoveArmToPose(self,ee_pose):
        # Moves to a defined Pose
        function_name = "MoveArmToPose"

        current_pose = self.arm.get_current_pose()

        ee_pose.orientation = current_pose.pose.orientation # remove this later when we solve for orientaiton

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

    # my_robot.MoveArmToTarget("ready")
    # # my_robot.MoveRailToTarget("right")
    # my_robot.MoveRail(0.1)

    # my_robot.MoveRail(1.2)


    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.w = 1.0
    pose_target.position.x = -1.37
    pose_target.position.y = -0.7
    pose_target.position.z = 0.45

    my_robot.MoveArmToPose(pose_target)

    
    
    moveit_commander.roscpp_shutdown()
        

        

    