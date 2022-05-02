#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
from std_msgs.msg import String



def main():
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous =True)
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()

        arm_group = moveit_commander.MoveGroupCommander("manipulator")
        rail = moveit_commander.MoveGroupCommander("rail")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        arm_group.set_named_target("ready")
        arm_group.go()

        rail.set_named_target("left")
        rail.go()

        rail.set_named_target("right")
        rail.go()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()