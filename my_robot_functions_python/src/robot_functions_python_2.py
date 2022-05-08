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
        


    def MoveArmToTarget(self,name):
        # moves to stored target

        status = False
        function_name = "MoveArmToTarget"

        #Set Target
        self.arm.set_named_target(name)

        #Plan
        success , plan = self._plan(self.arm, function_name)
        
        # Check Plan
        if success:
            # execute and check status
            success = self._execute(self.arm, plan, function_name) 

            if success:
                status = True # Function Succeeded
                self.arm.set_start_state_to_current_state()
            else:
                status = False # Function Failed
        else:
            status = False # Function Failed
        
        return status

    def MoveRailToTarget(self,name):
        # moves to stored target

        status = False
        function_name = "MoveRailToTarget"

        #Set Target
        self.rail.set_named_target(name)

        #Plan
        success , plan = self._plan(self.rail, function_name)
        
        # Check Plan
        if success:
            # execute and check status
            success = self._execute(self.rail, plan, function_name) 
            self.rail.set_start_state_to_current_state()

            if success:
                status = True # Function Succeeded
            else:
                status = True # Function Succeeded
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

    my_robot.MoveArmToTarget("ready")

    
    
    moveit_commander.roscpp_shutdown()
        

        

    