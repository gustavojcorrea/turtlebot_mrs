#!/usr/bin/env python

import rospy
import smach
import smach_ros
import tf

import threading
import actionlib

import numpy as np
from modern_robotics import *

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Point, Twist
from std_msgs.msg import Empty, Bool, Float64, Int16
from ar_track_alvar_msgs.msg import AlvarMarkers
from turtlebot_mrs_msgs.msg import RobotStatus
from math import pi, sqrt, atan2


DEBUG = True # True: output debug message, False: don't output debug message
RosActionLibEnable = True # True: enable ActionLib for sending goals
AutonomousMode = False #True: demo mode on, False: demo mode off

# define state IDLE
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu'],
                                    input_keys=['status_in'],
                                    output_keys=['status_out'])

        #self.robot_state_publisher = rospy.Publisher('/robot_state', RobotStatus, queue_size = 10)
        self.arduino_publisher = rospy.Publisher('/toggle_relay', Bool, queue_size = 10)
        self.electromagnet_state = Bool()


    def execute(self, userdata):
        rospy.loginfo('IDLE STATE: State Machine Has Started')

        # userdata.status_out = userdata.status_in
        # userdata.status_out.current_state = "init"
        robot_status_publish = userdata.status_in
        robot_status_publish.current_state = "init"
        userdata.status_out = robot_status_publish
        self.robot_state_publisher.publish(robot_status_publish)

	return 'to_menu'


# define state MENU
class Menu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['begin_approach','lock_magnet','get_artag_info','exit_sm'],
                                    input_keys=['status_in'],
                                    output_keys=['status_out'])

        # self.robot_state_publisher = rospy.Publisher('/robot_state', RobotStatus, queue_size = 10)
        # self.robot_state = RobotStatus()
        self.robot_state_publisher = rospy.Publisher('/robot_state', RobotStatus, queue_size = 10)

    def execute(self, userdata):
        rospy.loginfo('MENU STATE: In Menu')
        # self.robot_state.state = "MENU"
        # self.robot_state_publisher.publish(self.robot_state)
        #user_input = raw_input("insert test input: ")

        #robot_status_publish = userdata.status_in
        #robot_status_publish.current_state = "init"
        #userdata.status_out = robot_status_publish
        #self.robot_state_publisher.publish(robot_status_publish)

        user_input = raw_input("Press (1) to approach, (2) to Lock, (3) AR Tag (x) to exit state machine: ")



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
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'],
                                    input_keys=['status_in'],
                                    output_keys=['status_out'])

        self.frame_id = rospy.get_param('~goal_frame_id','robot_1/map')
	print(self.frame_id)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('robot_1/move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')

        if RosActionLibEnable:
            self.client.wait_for_server()

        rospy.loginfo('Connected to move_base.')

    def execute(self, userdata):
        rospy.loginfo('APPROACH STATE: In Approach state')

        user_input = raw_input("Press (1) to insert coordinates or (m) to go to menu: ")

        if user_input == "1":
            input_x = float(raw_input("Input x: "))
            input_y = float(raw_input("Input y: "))
            input_yaw_deg = float(raw_input("Input yaw(deg): "))

            input_yaw_rad = ( input_yaw_deg / 180 ) * pi
            input_q = tf.transformations.quaternion_from_euler(0, 0, input_yaw_rad)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.x = input_q[0]
            goal.target_pose.pose.orientation.y = input_q[1]
            goal.target_pose.pose.orientation.z = input_q[2]
            goal.target_pose.pose.orientation.w = input_q[3]

            rospy.loginfo('Executing move_base goal to position (x,y,yaw): %s, %s, %s' %
                    (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, input_yaw_deg))

            self.client.send_goal(goal)
            self.client.wait_for_result()

            return 'to_menu'
        else:
            return 'to_menu'

# define state PLAN
class Plan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'])
        # self.br = tf.TransformBroadcaster()
        # self.listener = tf.TransformListener()
        self.frame_id = rospy.get_param('~goal_frame_id','robot_1/map')
        self.client = actionlib.SimpleActionClient('robot_1/move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')

        if RosActionLibEnable:
            self.client.wait_for_server()

        rospy.loginfo('Connected to move_base.')


        self.rate = rospy.Rate(10.0)
        self.artag_subscriber = rospy.Subscriber('robot_1/ar_pose_marker', AlvarMarkers, self.artag_callback)
        self.ar_tag_markers  = []

    def artag_callback(self,msg):
        self.ar_tag_markers = msg.markers

    def execute(self, userdata):
        rospy.loginfo('PLAN STATE: Planning trajectory')

        user_input = raw_input("Press (1) Get AR Tag Location (2) Menu (3) Exit: ")


        if user_input == '1':
            #rospy.loginfo('Displaying (x,y,z): %s, %s, %s' %
             #       (ar_tag_markers.markers[0].pose.pose.psosition.x,ar_tag_markers.markers[0].pose.pose.position.y,ar_tag_markers.markers[0].pose.pose.position.z))
            #rospy.loginfo('Displaying (x,y,z): %s, %s, %s' %
            #        (self.ar_tag_markers.markers[0].pose.pose.position.x,self.ar_tag_markers.markers[0].pose.pose.position.y,self.ar_tag_markers.markers[0].pose.pose.position.z))
            while not rospy.is_shutdown():
                # input_yaw_rad = 1.57
                # q = tf.transformations.quaternion_from_euler(0, 0, input_yaw_rad)
                # self.br.sendTransform((.3, 0.0, 0.0),(q[0],q[1],q[2],q[3]),rospy.Time.now(),"robot_2","robot_1/camera_link")
                rospy.loginfo('Executing move_base goal to position (x,y,yaw): %s, %s, %s' %
                        (self.ar_tag_markers[0].pose.pose.position.x,self.ar_tag_markers[0].pose.pose.position.y,self.ar_tag_markers[0].pose.pose.position.z))
                self.rate.sleep()

            return 'to_menu'
        elif user_input == '2':
            listener = tf.TransformListener()



            #listener.waitForTransform('robot_1/map', 'ar_marker_1', rospy.Time(), rospy.Duration(10.0))
            rate = rospy.Rate(10.0)
            while not rospy.is_shutdown():
                try:
                    #now = rospy.Time.now()
                    time = rospy.Time(0)
                    (transMC,rotMC) = listener.lookupTransform('robot_1/map', 'robot_1/camera_link', time)
                    (transMR,rotMR) = listener.lookupTransform('robot_1/map', 'robot_1/base_link', time)
                    (transCA,rotCA) = listener.lookupTransform('robot_1/camera_link', 'ar_marker_1', time)

                    #listener.waitForTransform('robot_1/map', 'ar_marker_1', now, rospy.Duration(10.0))
                    #(trans,rot) = listener.lookupTransform('robot_1/map', 'ar_marker_1', now)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                rospy.loginfo('Transform from Robot to Map (x,y,z): %s, %s, %s' %
                        (transMR[0],transMR[1],transMR[2]))
                rospy.loginfo('Transform from Camera to Map (x,y,z): %s, %s, %s' %
                        (transMC[0],transMC[1],transMC[2]))
                rospy.loginfo('Transform from AR to Camera (x,y,z): %s, %s, %s' %
                        (transCA[0],transCA[1],transCA[2]))

                break

            test1 = np.array([(1.5,2,3,1), (4,5,6,1),(4,5,6,1),(4,5,6,1)])
            user_input = raw_input("Press (1) Move Robot (2) Menu: ")

            if user_input == '1':
            	# input_yaw_rad = 0;
            	# input_q = tf.transformations.quaternion_from_euler(0, 0, input_yaw_rad)
                #
            	# goal = MoveBaseGoal()
            	# goal.target_pose.header.frame_id = self.frame_id
            	# goal.target_pose.pose.position.x = trans[2]  + 0.08
            	# goal.target_pose.pose.position.y = -trans[1] - 0.30
            	# goal.target_pose.pose.position.z = trans[0]
            	# goal.target_pose.pose.orientation.x = input_q[0]
            	# goal.target_pose.pose.orientation.y = input_q[1]
            	# goal.target_pose.pose.orientation.z = input_q[2]
            	# goal.target_pose.pose.orientation.w = input_q[3]
                #
            	# rospy.loginfo('Executing move_base goal to position (x,y,z): %s, %s, %s' %
                #     (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z))
                #
            	# self.client.send_goal(goal)
            	# self.client.wait_for_result()
				return 'to_menu'
            else:
                return 'to_menu'
		return 'to_menu'


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
    sm_top = smach.StateMachine(outcomes=['End'])
    sm_top.userdata.sm_status = RobotStatus()

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'to_menu':'MENU'},
                               remapping={'status_in':'sm_status',
                                          'status_out':'sm_status'})
        smach.StateMachine.add('MENU', Menu(),
                               transitions={'begin_approach':'APPROACH','lock_magnet':'LOCK','get_artag_info':'PLAN','exit_sm':'End'},
                               remapping={'menu_status_in':'sm_status',
                                          'menu_status_out':'sm_status'})
        smach.StateMachine.add('APPROACH', Approach(),
                               transitions={'to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('LOCK', Lock(),
                               transitions={'to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('PLAN', Plan(),
                               transitions={'to_menu':'MENU','exit_sm':'End'})

    #Create and start introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
