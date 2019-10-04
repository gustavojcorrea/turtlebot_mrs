#!/usr/bin/env python

# import rospy
# import smach
# import smach_ros
# import tf
#
# import threading
# import actionlib
#
# import numpy as np
from turtlebot_move import *

# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Point, Twist
# from std_msgs.msg import Empty, Bool, Float64, Int16
# from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
# from turtlebot_mrs_msgs.msg import RobotStatus
# from math import pi, sqrt, atan2


DEBUG = True # True: output debug message, False: don't output debug message
RosActionLibEnable = False # True: enable ActionLib for sending goals
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

	return 'to_menu'


# define state MENU
class Menu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['begin_approach','lock_magnet','get_artag_info','exit_sm','move'],
                                    input_keys=['status_in'],
                                    output_keys=['status_out'])

        # self.robot_state_publisher = rospy.Publisher('/robot_state', RobotStatus, queue_size = 10)
        # self.robot_state = RobotStatus()
        self.robot_state_publisher = rospy.Publisher('/robot_state', RobotStatus, queue_size = 10)

    def execute(self, userdata):
        rospy.loginfo('MENU STATE: In Menu')

        user_input = raw_input("Choose Option: \n (1) Demo Mode (2) Demo Mode with User Input \n (3) Approach (4) Magnet Control (5) Output AR Tag (6)Move (7) to exit state machine: ")

        if   user_input == '1':
	        return 'begin_approach'
        elif user_input == '2':
	        return 'begin_approach'
        elif user_input == '3':
	        return 'begin_approach'
        elif user_input == '4':
	        return 'lock_magnet'
        elif user_input == '5':
	        return 'get_artag_info'
        elif user_input == "6":
            return "move"
        else:
            return 'exit_sm'

# define state APPROACH
class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'],
                                    input_keys=['status_in'],
                                    output_keys=['status_out'])

        #START: Connect to move_base node for NavStack waypoint navigation #############################################
        self.frame_id = rospy.get_param('~goal_frame_id','robot_1/map')

        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('robot_1/move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')

        # RosActionLibEnable is set to true if in debug mode and robot_1 is not on
        if RosActionLibEnable:
            self.client.wait_for_server()

        rospy.loginfo('Connected to move_base.')
        #END: Connect to move_base node for NavStack waypoint navigation #############################################


    def execute(self, userdata):
        rospy.loginfo('APPROACH STATE: In Approach state')

        if AutonomousMode == False:
            user_input = raw_input("Press (1) to insert coordinates or (m) to go to menu: ")
        elif AutonomousMode == True:
            user_input = "Autonomous"

        if user_input == "1":

            #Get waypoint from user input #############################################
            input_x = float(raw_input("Input x: "))
            input_y = float(raw_input("Input y: "))
            input_yaw_deg = float(raw_input("Input yaw (deg, default = 0): "))

            #Change yaw from degrees to radians and store into a quarternion #############################################
            input_yaw_rad = ( input_yaw_deg / 180 ) * pi
            input_q = tf.transformations.quaternion_from_euler(0, 0, input_yaw_rad)

            #Create MoveBaseGoal from user input and send to navigation stack #############################################
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y
            goal.target_pose.pose.position.z = 0.05199 # Set to the Z value or height with respect to ground of base_link
            goal.target_pose.pose.orientation.x = input_q[0]
            goal.target_pose.pose.orientation.y = input_q[1]
            goal.target_pose.pose.orientation.z = input_q[2]
            goal.target_pose.pose.orientation.w = input_q[3]

            rospy.loginfo('Executing move_base goal to position (x,y,yaw): %s, %s, %s' %
                    (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, input_yaw_deg))

            self.client.send_goal(goal)
            self.client.wait_for_result()

            return 'to_menu'

        elif user_input == "Autonomous":

            #Get waypoint from user input #############################################
            input_x = 3.0
            input_y = 3.0
            input_yaw_deg = 0.0

            #Change yaw from degrees to radians and store into a quarternion #############################################
            input_yaw_rad = ( input_yaw_deg / 180 ) * pi
            input_q = tf.transformations.quaternion_from_euler(0, 0, input_yaw_rad)

            #Create MoveBaseGoal from user input and send to navigation stack #############################################
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position.x = input_x
            goal.target_pose.pose.position.y = input_y
            goal.target_pose.pose.position.z = 0.05199 # Set to the Z value or height with respect to ground of base_link
            goal.target_pose.pose.orientation.x = input_q[0]
            goal.target_pose.pose.orientation.y = input_q[1]
            goal.target_pose.pose.orientation.z = input_q[2]
            goal.target_pose.pose.orientation.w = input_q[3]

            rospy.loginfo('AUTONOMOUS MODE: Executing move_base goal to position (x,y,yaw): %s, %s, %s' %
                    (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, input_yaw_deg))

            self.client.send_goal(goal)
            self.client.wait_for_result()

            return 'to_menu' # FIXME: SET CORRECT TRANSITION
        else:
            return 'to_menu'

# define state PLAN
class Plan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_docking','to_menu','exit_sm'],
                                    output_keys=['plan_msg_out'])

        self.rate = rospy.Rate(10.0)

        # INIT SUBSCRIBERS
        self.artag_subscriber = rospy.Subscriber('robot_1/ar_pose_marker', AlvarMarkers, self.artag_callback)
        self.ar_tag_markers  = []

        # INIT PUBLISHERS
        self.ar_tag_publisher = rospy.Publisher('/robot_2/ar_tag_position',AlvarMarker, queue_size = 10)
        self.ar_tag_position = AlvarMarker()

    def artag_callback(self,msg):
        self.ar_tag_markers = msg.markers

    def execute(self, userdata):
        rospy.loginfo('PLAN STATE: Planning trajectory')

        if AutonomousMode == False:
            user_input = raw_input("Press (1) View AR Tag Output (2) Menu (3) Exit:  ")
        elif AutonomousMode == True:
            user_input == "Autonomous"


        if user_input == "Autonomous":

            rospy.loginfo('Publishing AR Tag position')

            while not rospy.is_shutdown():

                if len(self.ar_tag_markers) != 0: # Doesn't display AR markers until the marker is in line of sight
                    AR_TAG_NUM = 0
                    # rospy.loginfo('AR Tag Position RAW Value (x,y,z): %s, %s, %s' %
                    #         (self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.x,self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.y,self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.z))
                    # rospy.loginfo('AR Tag Position CORRECTED Value (x,y,z): %s, %s, %s' %
                    #         ( self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.z, -self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.x, -self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.y))

                    self.ar_tag_position = self.ar_tag_markers[AR_TAG_NUM]

                    # Correct output of AR Tag on Robot_2 in respect to camera on Robot_1 (with camera)

                    x = self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.x
                    y = self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.y
                    z = self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.z
                    self.ar_tag_position.pose.pose.position.x = z
                    self.ar_tag_position.pose.pose.position.y = -x
                    self.ar_tag_position.pose.pose.position.z = -y

                self.ar_tag_publisher.publish(self.ar_tag_position)
                userdata.plan_msg_out = self.ar_tag_position
                self.rate.sleep()
            return 'to_menu'

        elif user_input == '1':

            rospy.loginfo('Publishing AR Tag position')

            got_ar_tag = False
            # while not rospy.is_shutdown():
            while got_ar_tag == False:

                rospy.sleep(2.0)
                if len(self.ar_tag_markers) != 0: # Doesn't display AR markers until the marker is in line of sight
                    AR_TAG_NUM = 0
                    # rospy.loginfo('AR Tag Position RAW Value (x,y,z): %s, %s, %s' %
                    #         (self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.x,self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.y,self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.z))
                    # rospy.loginfo('AR Tag Position CORRECTED Value (x,y,z): %s, %s, %s' %
                    #         ( self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.z, -self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.x, -self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.y))

                    self.ar_tag_position = self.ar_tag_markers[AR_TAG_NUM]

                    # Correct output of AR Tag on Robot_2 in respect to camera on Robot_1 (with camera)

                    x = self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.x
                    y = self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.y
                    z = self.ar_tag_markers[AR_TAG_NUM].pose.pose.position.z
                    self.ar_tag_position.pose.pose.position.x = z
                    self.ar_tag_position.pose.pose.position.y = -x
                    self.ar_tag_position.pose.pose.position.z = -y

                    self.ar_tag_publisher.publish(self.ar_tag_position)
                    userdata.plan_msg_out = self.ar_tag_position

                    rospy.loginfo('Corrected AR Tag position (x,y,z): %s, %s, %s ' %
                            (self.ar_tag_position.pose.pose.position.x, self.ar_tag_position.pose.pose.position.y,self.ar_tag_position.pose.pose.position.z))
                    got_ar_tag = True
                self.rate.sleep()

            return 'start_docking'

        elif user_input == '2':
            return 'to_menu'

        elif user_input == '3':
            return 'exit_sm'

            return 'exit_sm'

# define state DOCK
class Dock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'],
                                    input_keys=['dock_msg_in'])

        self.rate = rospy.Rate(10.0)

        # INIT SUBSCRIBERS
        # self.artag_subscriber = rospy.Subscriber('robot_1/ar_pose_marker', AlvarMarkers, self.artag_callback)
        # self.ar_tag_markers  = []

        # INIT PUBLISHERS
        self.arduino_publisher = rospy.Publisher('/toggle_relay', Bool, queue_size = 10)
        self.electromagnet_state = Bool()

    # def ??_callback(self,msg):
    #     self.ar_tag_markers = msg.markers

    def execute(self, userdata):
        rospy.loginfo('DOCK STATE: Docking robots')


        if AutonomousMode == False:
            rospy.loginfo('!!!MAGNET OFF!!')
            self.arduino_publisher.publish(1)
            user_input = raw_input("Press (1) Dock (2) Menu (3) Exit:  ")
        elif AutonomousMode == True:
            user_input == "Autonomous"

        if user_input == "Autonomous":

            rospy.loginfo('Autonomous')

        elif user_input == '1':
            rospy.loginfo('Docking started')
            rospy.loginfo('!!!MAGNET ON!!')
            self.arduino_publisher.publish(0)

            Xar = userdata.dock_msg_in.pose.pose.position.x # x distance from AR tag to camera_link of robot_1
            Yar = userdata.dock_msg_in.pose.pose.position.y # y distance from AR tag to camera_link of robot_1
            rospy.loginfo('Destination waypoint for robot_1 is: %s, %s ' %
                    (Xar, Yar))

            turtlebot_move(Xar, Yar)

            return 'to_menu'

        elif user_input == '2':
            return 'to_menu'

        elif user_input == '3':
            return 'exit_sm'

            return 'exit_sm'


# define state LOCK
class Lock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'])
        self.arduino_publisher = rospy.Publisher('/toggle_relay', Bool, queue_size = 10)
        self.electromagnet_state = Bool()
    def execute(self, userdata):
        rospy.loginfo('LOCK STATE: Begin Locking')

        user_input = raw_input("Press (1) to Unlock, (2) to Lock, (3) Try again (x) to exit state machine: ")

        if user_input == '1':
            self.electromagnet_state = 1 # Publish 1 to turn magnet off
            self.arduino_publisher.publish(1)
            return 'to_menu'
        elif user_input == '2':
            self.electromagnet_state = 0 # Publish 1 to turn magnet on
            self.arduino_publisher.publish(0)
            return 'to_menu'
        else:
            return 'exit_sm'
# main

# define state Move
class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_undocking','to_menu','exit_sm'])

        # INIT SUBSCRIBERS
        self.keyop_subscriber = rospy.Subscriber('multi/mobile_base/commands/velocity', Twist, self.keyop_callback)
        self.keyop_velocity = Twist()

        self.motor_power_subscriber = rospy.Subscriber('/multi/mobile_base/commands/motor_power', MotorPower, self.motor_power_callback)
        self.motor_power_status = MotorPower()

        # INIT PUBLISHERS
        self.robot_1_velocity = rospy.Publisher('robot_1/mobile_base/commands/velocity', Twist, queue_size = 1)
        self.robot_2_velocity = rospy.Publisher('robot_2/mobile_base/commands/velocity', Twist, queue_size = 1)


    def keyop_callback(self,msg):
        self.keyop_velocity = msg

    def motor_power_callback(self,msg):
        self.motor_power_status = msg

    def execute(self, userdata):
        rospy.loginfo('MOVE STATE: Moving robots')

        rate = rospy.Rate(100)

        rospy.loginfo('If motors disabled, press (e) to enable moto')
        while not (self.motor_power_status.state == 1):
            # rospy.loginfo(self.motor_power_status.state)
            if self.motor_power_status.state == 1:
                rospy.loginfo('Motos enabled')
                # break

        rospy.loginfo('Multi move begin')
        while self.motor_power_status.state == 1:
            # rospy.loginfo('Linear Velocity (x,y,z): %s, %s, %s ' %
            #         (self.keyop_velocity.linear.x, self.keyop_velocity.linear.y,self.keyop_velocity.linear.z))

            center_distance = 0.432
            V_xy = self.keyop_velocity.linear.x
            omega = self.keyop_velocity.angular.z

            if omega != 0:
                icc_radius = V_xy / omega
                V_l = omega * ( icc_radius - center_distance / 2) # Robot_2 tangential velocity
                V_r = omega * ( icc_radius + center_distance / 2) # Robot_1 tangential velocity
            else:
                V_l = V_xy # Robot_2 tangential velocity
                V_r = V_xy # Robot_1 tangential velocity

            robot_1_vel = Twist()
            robot_2_vel = Twist()

            robot_1_vel.linear.x = V_r
            robot_2_vel.linear.x = V_l

            robot_1_vel.angular.z = 0
            robot_2_vel.angular.z = 0

            self.robot_1_velocity.publish(robot_1_vel.linear,robot_1_vel.angular)
            self.robot_2_velocity.publish(robot_2_vel.linear,robot_2_vel.angular)


        if AutonomousMode == False:
            user_input = raw_input("Press (1) Undock robots (2) Menu (3) Exit:  ")
        elif AutonomousMode == True:
            user_input == "Autonomous"

        if user_input == "Autonomous":

            rospy.loginfo('Autonomous')

        elif user_input == '1':

            return 'start_undocking'

        elif user_input == '2':
            return 'to_menu'

        elif user_input == '3':
            return 'exit_sm'

        return 'exit_sm'

# define state UNDOCK
class Undock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'])

        self.rate = rospy.Rate(100.0)

        # INIT SUBSCRIBERS
        # self.artag_subscriber = rospy.Subscriber('robot_1/ar_pose_marker', AlvarMarkers, self.artag_callback)
        # self.ar_tag_markers  = []

        # INIT PUBLISHERS
        self.arduino_publisher = rospy.Publisher('/toggle_relay', Bool, queue_size = 10)
        self.electromagnet_state = Bool()

        self.robot_1_velocity = rospy.Publisher('robot_1/mobile_base/commands/velocity', Twist, queue_size = 1)
        self.robot_2_velocity = rospy.Publisher('robot_2/mobile_base/commands/velocity', Twist, queue_size = 1)

    # def ??_callback(self,msg):
    #     self.ar_tag_markers = msg.markers
    def my_callback(self,event):
        print 'Timer called at ' + str(event.current_real)


    def execute(self, userdata):
        rospy.loginfo('UNDOCK STATE: Undocking robots')

        now = rospy.get_time()
        start_time = now

        publish_time = 2

        self.electromagnet_state = 1 # Publish 1 to turn magnet off
        self.arduino_publisher.publish(self.electromagnet_state)
        rospy.loginfo('MAGNET OFF!!!')

        while (now - start_time) < publish_time:
            rospy.loginfo(now - start_time)
            robot_1_vel = Twist()
            robot_2_vel = Twist()

            robot_1_vel.linear.x = -0.2
            robot_2_vel.linear.x = 0.2

            robot_1_vel.angular.z = 0
            robot_2_vel.angular.z = 0

            self.robot_1_velocity.publish(robot_1_vel.linear,robot_1_vel.angular)
            # self.robot_2_velocity.publish(robot_2_vel.linear,robot_2_vel.angular)

            now = rospy.get_time()

        rospy.loginfo('UNDOCK STATE: FINISHED Undocking robots')





        if AutonomousMode == False:
            user_input = raw_input("Press (1) ## (2) Menu (3) Exit:  ")
        elif AutonomousMode == True:
            user_input == "Autonomous"

        if user_input == "Autonomous":

            rospy.loginfo('Autonomous')

        elif user_input == '1':

            return 'to_menu'

        elif user_input == '2':
            return 'to_menu'

        elif user_input == '3':
            return 'exit_sm'

            return 'exit_sm'





def main():
    rospy.init_node('mrscot_state_machine')

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['End'])
    sm_top.userdata.sm_status = RobotStatus()
    sm_top.userdata.ar_tag_pos = AlvarMarker()

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'to_menu':'MENU'},
                               remapping={'status_in':'sm_status',
                                          'status_out':'sm_status'})
        smach.StateMachine.add('MENU', Menu(),
                               transitions={'begin_approach':'APPROACH','lock_magnet':'LOCK','get_artag_info':'PLAN','move':'MOVE','exit_sm':'End'},
                               remapping={'menu_status_in':'sm_status',
                                          'menu_status_out':'sm_status'})
        smach.StateMachine.add('APPROACH', Approach(),
                               transitions={'to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('LOCK', Lock(),
                               transitions={'to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('PLAN', Plan(),
                               transitions={'start_docking':'DOCK','to_menu':'MENU','exit_sm':'End'},
                               remapping={'plan_msg_out':'ar_tag_pos'})
        smach.StateMachine.add('DOCK', Dock(),
                               transitions={'to_menu':'MENU','exit_sm':'End'},
                               remapping={'dock_msg_in':'ar_tag_pos'})
        smach.StateMachine.add('MOVE', Move(),
                               transitions={'start_undocking':'UNDOCK','to_menu':'MENU','exit_sm':'End'})
        smach.StateMachine.add('UNDOCK', Undock(),
                               transitions={'to_menu':'MENU','exit_sm':'End'})

    #Create and start introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()

    # unlock magnet before exiting program
    # arduino_publisher = rospy.Publisher('/toggle_relay', Bool, queue_size = 10)
    # arduino_publisher.publish(1)

    sis.stop()

if __name__ == '__main__':
    main()



# define state PLAN
# class Plan(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['to_menu','exit_sm'],
#                                     output_keys=['plan_msg_out'])
#
#         self.rate = rospy.Rate(10.0)
#
#         # INIT SUBSCRIBERS
#         self.artag_subscriber = rospy.Subscriber('robot_1/ar_pose_marker', AlvarMarkers, self.artag_callback)
#         self.ar_tag_markers  = []
#
#         # INIT PUBLISHERS
#         self.ar_tag_publisher = rospy.Publisher('/robot_2/ar_tag_position',AlvarMarker, queue_size = 10)
#         self.ar_tag_position = AlvarMarker()
#
#     def ??_callback(self,msg):
#         self.ar_tag_markers = msg.markers
#
#     def execute(self, userdata):
#         rospy.loginfo('PLAN STATE: Planning trajectory')
#
#         if AutonomousMode == False:
#             user_input = raw_input("Press (1) ## (2) Menu (3) Exit:  ")
#         elif AutonomousMode == True:
#             user_input == "Autonomous"
#
#         if user_input == "Autonomous":
#
#             rospy.loginfo('Autonomous')
#
#         elif user_input == '1':
#
#             rospy.loginfo('Docking')
#
#             return 'to_menu'
#
#         elif user_input == '2':
#             return 'to_menu'
#
#         elif user_input == '3':
#             return 'exit_sm'
#
#             return 'exit_sm'
