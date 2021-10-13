#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float32
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState




length = 0

   	

def joint_plan(square_length):

	

	global length, move_group
	length = square_length.data
	print length
	

	joint_goal = move_group.get_current_joint_values()
	joint_goal[0] = 0
	joint_goal[1] = -pi/4
	joint_goal[2] = 0
	joint_goal[3] = -pi/2
	joint_goal[4] = 0
	joint_goal[5] = pi/3
	joint_goal[6] = 0

	move_group.go(joint_goal, wait=True)
	move_group.stop()
	print "Moved to Initial Position"
	rospy.sleep(1)

	cartesian_plan, fraction = plan_cartesian()
	print "Path Planned, Displaying"
	rospy.sleep(3)

	print "Executing Plan"
	execution(cartesian_plan)
	
	rospy.sleep(3)
	print "Success, waiting for new message"


def plan_cartesian():
	
	global length
	waypoints = []
	
	
    	wpose = move_group.get_current_pose().pose
 	wpose.position.z -= 0.1  # First move up z
    	wpose.position.x +=0 # Move to start of square in x
	wpose.position.y +=0 # Move to start of square in y
	waypoints.append(copy.deepcopy(wpose))

    	wpose.position.z += 0  # Move down in z
	wpose.position.x +=length # Move to right of square in x
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.y +=length   # Move to top of square in y
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.x -=length # Move to left of square in x
        waypoints.append(copy.deepcopy(wpose))

	wpose.position.y -=length   # Move to bottom of square in y 
        waypoints.append(copy.deepcopy(wpose))

	(plan, fraction) = move_group.compute_cartesian_path( waypoints,   # waypoints to follow
        	                               				0.01,        # eef_step
                	                       				0.0)         # jump_threshold

	return plan, fraction

	display_trajectory(plan)
	print "Plan Complete"

def display_trajectory(plan):

		
	global robot, display_trajectory_publisher

	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
	# Publish
	display_trajectory_publisher.publish(display_trajectory);

def execution(plan):

	global move_group 
	move_group.execute(plan, wait=True)
	

def plan_path():

	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
	print "Waiting for Message"
	rospy.wait_for_message('square_size', Float32)
    	sub = rospy.Subscriber("square_size", Float32, joint_plan)

if __name__ == '__main__':

	moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('move_panda_square', anonymous=True)


	robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"

        move_group = moveit_commander.MoveGroupCommander(group_name)

	planning_frame = move_group.get_planning_frame()
    	print "============ Planning frame: %s" % planning_frame

  	# We can also print the name of the end-effector link for this group:
  	eef_link = move_group.get_end_effector_link()
  	print "============ End effector link: %s" % eef_link

    	# We can get a list of all the groups in the robot:
    	group_names = robot.get_group_names()
    	print "============ Available Planning Groups:", robot.get_group_names()

    	# Sometimes for debugging it is useful to print the entire state of the
    	# robot:
    	print "============ Printing robot state"
    	print robot.get_current_state()
    	print ""
			
	plan_path()
		
	rospy.spin()
	

