#!/usr/bin/env python

import rospy
import smach
import smach_ros

import threading
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Point, Twist
from std_msgs.msg import Empty, Bool, Float64, Int16
from ar_track_alvar_msgs.msg import AlvarMarkers
from turtlebot_mrs_msgs.msg import RobotStatus

# define state IDLE
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu'])



        self.arduino_publisher = rospy.Publisher('/toggle_relay', Bool, queue_size = 10)
        self.electromagnet_state = Bool()

        self.artag_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.artag_callback)
        self.ar_tag_markers  = AlvarMarkers()

    def artag_callback(self,msg):
        self.ar_tag_markers = msg

    def execute(self, userdata):
        rospy.loginfo('IDLE STATE: State Machine Has Started')
	return 'to_menu'


# define state MENU
class Menu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['begin_approach','lock_magnet','get_artag_info','exit_sm'])

        self.robot_state_publisher = rospy.Publisher('/robot_state', RobotStatus, queue_size = 10)
        self.robot_state = RobotStatus()

    def execute(self, userdata):
        rospy.loginfo('MENU STATE: In Menu')
        #user_input = raw_input("insert test input: ")
        user_input = raw_input("Press (1) to approach, (2) to Lock, (3) AR Tag (x) to exit state machine: ")

        self.robot_state.start_all_nodes = "yes"
        self.robot_state_publisher.publish(self.robot_state)

        if user_input == '1':
	        return 'begin_approach'
        elif user_input == '2':
	        return 'lock_magnet'
        elif user_input == '3':
	        return 'get_artag_info'
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
        #self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def execute(self, userdata):
        rospy.loginfo('APPROACH STATE: In Approach state')

        user_input = raw_input("Press (1) to insert coordinates or (m) to go to menu: ")

        if user_input == "1":
            input_x = float(raw_input("Input x: "))
            input_y = float(raw_input("Input y: "))
            input_w = float(raw_input("Input w: "))

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
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'])

        # self.artag_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.artag_callback)
        # self.ar_tag_markers  = AlvarMarkers()

    # def artag_callback(self,msg):
    #     self.ar_tag_markers = msg

    def execute(self, userdata):
        rospy.loginfo('PLAN STATE: Planning trajectory')

        user_input = raw_input("Press (1) Get AR Tag Location (2) Menu (3) Exit: ")
        if user_input == '1':
            #rospy.loginfo('Displaying (x,y,z): %s, %s, %s' %
             #       (ar_tag_markers.markers[0].pose.pose.position.x,ar_tag_markers.markers[0].pose.pose.position.y,ar_tag_markers.markers[0].pose.pose.position.z))
            rospy.loginfo('Displaying (x,y,z): %s, %s, %s' %
                    (self.ar_tag_markers.markers[0].pose.pose.position.x,self.ar_tag_markers.markers[0].pose.pose.position.y,self.ar_tag_markers.markers[0].pose.pose.position.z))
            return 'to_menu'
        elif user_input == '2':
	        return 'to_menu'
        else:
            return 'exit_sm'

# define state LOCK
class Lock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'])
        # self.arduino_publisher = rospy.Publisher('/toggle_relay', Bool, queue_size = 10)
        # self.electromagnet_state = Bool()
    def execute(self, userdata):
        rospy.loginfo('LOCK STATE: Begin Locking')

        user_input = raw_input("Press (1) to Lock, (2) to Unlock, (3) Try again (x) to exit state machine: ")

        if user_input == '1':
            self.electromagnet_state = 1
            self.arduino_publisher.publish(1)
            return 'to_menu'
        elif user_input == '2':
            self.electromagnet_state = 0
            self.arduino_publisher.publish(0)
            return 'to_menu'
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
                               transitions={'begin_approach':'APPROACH','lock_magnet':'LOCK','get_artag_info':'PLAN','exit_sm':'End'})
        smach.StateMachine.add('APPROACH', Approach(),
                               transitions={'to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('LOCK', Lock(),
                               transitions={'to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('PLAN', Plan(),
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
