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
        
        self.arm.allow_replanning(True)
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        


    def MoveArmToTarget(self,name):
        # moves to stored target

        status = False

        #Set Target
        self.arm.set_named_target(name)

        #Plan
        error_code_val, plan, planning_time, error_code  = self.arm.plan()

        # Get status
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        
        # Check Plan
        if success:
            # execute
            error_code_val = self.arm.execute(plan)

            # Get status
            success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

            if success:
                status = True # Function Succeeded
            else:
                status = False # Function Failed
        else:
            status = False # Function Failed
        
        return status


    


if __name__ == "__main__":
    rospy.loginfo("Grasp Manipulator On ...")
    my_robot = ManipulationFunctions()

    my_robot.MoveArmToTarget("ready")

    
    
    moveit_commander.roscpp_shutdown()
        

        

    