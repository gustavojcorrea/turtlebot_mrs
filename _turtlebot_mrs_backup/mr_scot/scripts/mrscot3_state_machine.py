#!/usr/bin/env python

import rospy
import smach
import smach_ros

import threading
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import Empty

# define state IDLE
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu'])

       


    def execute(self, userdata):
        rospy.loginfo('IDLE STATE: State Machine Has Started')
	return 'to_menu'


# define state MENU
class Menu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['begin_approach','exit_sm'])

    def execute(self, userdata):
        rospy.loginfo('MENU STATE: In Menu')
        #user_input = raw_input("insert test input: ")
        user_input = raw_input("Press (1) to approach, (x) to exit state machine: ")
        if user_input == '1':
	        return 'begin_approach'
        else:
            return 'exit_sm'
        
# define state APPROACH
class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'])

        self.frame_id = rospy.get_param('~goal_frame_id','robot_1/map')
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('robot_1/move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def execute(self, userdata):
        rospy.loginfo('APPROACH STATE: In Approach state')
	    
        user_input = raw_input("Press (1) to insert coordinates or (m) to go to menu: ")
	    
        if user_input == "1":
            input_x = float(raw_input("Input x: "))
            input_y = float(raw_input("Input y: "))
            input_w = float(raw_input("Input w "))

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.w = input_w
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
            #rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)
            self.client.wait_for_result()
            
            return 'to_menu'
        else:
            return 'to_menu'

# define state PLAN
class Plan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['execute_plan','to_menu','exit_sm'])

    def execute(self, userdata):
        rospy.loginfo('PLAN STATE: Planning trajectory')

        user_input = raw_input("Press (1) Get AR Tag Location (2) Menu (3) Exit: ")
        if user_input == '1':
	        return 'to_menu'
        elif user_input == '2':
	        return 'to_menu'
        else:
            return 'exit_sm'
 
# define state EXECUTE
class Execute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['execute_plan','exit_sm'])

    def execute(self, userdata):
        rospy.loginfo('EXECUTE STATE: Planning trajectory')

        user_input = raw_input("Press (b) to begin approach state or (x) to exit state machine: ")
        if user_input == 'b':
	        return 'begin_approach'
        else:
            return 'exit_sm'        

# main
def main():
    rospy.init_node('mrscot_state_machine')
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['End'])
    sm.userdata.sm_if_shutdown = 'n'
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'to_menu':'MENU'})
        smach.StateMachine.add('MENU', Menu(), 
                               transitions={'begin_approach':'APPROACH','exit_sm':'End'})
        smach.StateMachine.add('APPROACH', Approach(), 
                               transitions={'to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('PLAN', Plan(), 
                               transitions={'execute_plan':'EXECUTE','to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('EXECUTE', Execute(), 
                               transitions={'to_menu':'MENU','exit_sm':'End'})
        
    #Create and start introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
