#define state FACE
class FaceCV(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu','exit_sm'])


        # INIT SUBSCRIBERS
        self.faceCV_xaxis_subscriber = rospy.Subscriber('/Face_xaxis', Int16, self.faceCV_xaxis_callback)
        self.faceCV_xaxis = Int16()

        self.faceCV_yaxis_subscriber = rospy.Subscriber('/Face_yaxis', Int16, self.faceCV_yaxis_callback)
        self.faceCV_yaxis = Int16()

        # INIT PUBLISHERS
        self.robot_1_velocity = rospy.Publisher('robot_1/mobile_base/commands/velocity', Twist, queue_size = 1)
        self.sm_state_publisher = rospy.Publisher('/sm_state', String, queue_size = 5)

    def faceCV_xaxis_callback(self,msg):
        self.faceCV_xaxis = msg.data

    def faceCV_yaxis_callback(self,msg):
        self.faceCV_yaxis = msg.data

    def execute(self, userdata):
        self.sm_state_publisher.publish('HOME')
        rospy.loginfo('HOME STATE: Going Home')

        if AutonomousMode == False:
            user_input = raw_input("Press (1) View FaceCV input (2) Menu (3) Exit:  ")
        elif AutonomousMode == True:
            user_input == "Autonomous"

        if user_input == "Autonomous":

            rospy.loginfo('Autonomous')

        elif user_input == '1':

            rospy.Rate(100)
            while not rospy.is_shutdown():
                # rospy.loginfo('(x,y): %s, %s' %
                #         (self.faceCV_xaxis, self.faceCV_yaxis))

                robot_1_vel = Twist()
                # robot_1_vel.linear.x = 0
                # robot_1_vel.angular.z = 0
                if self.faceCV_yaxis == 0 and self.faceCV_xaxis == 0:
                    robot_1_vel.linear.x = 0.0
                    robot_1_vel.angular.z = 0
                    self.robot_1_velocity.publish(robot_1_vel.linear,robot_1_vel.angular)
                    rospy.loginfo('Stop')
                elif self.faceCV_yaxis < 200:
                    if (self.faceCV_xaxis > 133) and (self.faceCV_xaxis < 266):
                    #straight
                        robot_1_vel.linear.x = 0.3
                        robot_1_vel.angular.z = 0
                        self.robot_1_velocity.publish(robot_1_vel.linear,robot_1_vel.angular)
                        rospy.loginfo('Straight')

                    if self.faceCV_xaxis < 133:
                    #turn right
                         robot_1_vel.linear.x = 0.1
                         robot_1_vel.angular.z = -0.3
                         self.robot_1_velocity.publish(robot_1_vel.linear,robot_1_vel.angular)
                         rospy.loginfo('Right')

                    if self.faceCV_xaxis > 266:
                    #turn left
                        robot_1_vel.linear.x = 0.1
                        robot_1_vel.angular.z = 0.3
                        self.robot_1_velocity.publish(robot_1_vel.linear,robot_1_vel.angular)
                        rospy.loginfo('Left')

                elif self.faceCV_yaxis > 200:
                    robot_1_vel.linear.x = 0.0
                    robot_1_vel.angular.z = 0
                    self.robot_1_velocity.publish(robot_1_vel.linear,robot_1_vel.angular)
                    rospy.loginfo('Stop')

                # self.robot_1_velocity.publish(robot_1_vel.linear,robot_1_vel.angular)

            return 'to_menu'

        elif user_input == '2':
            return 'to_menu'

        elif user_input == '3':
            return 'exit_sm'

            return 'exit_sm'
