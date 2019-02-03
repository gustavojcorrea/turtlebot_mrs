#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Init
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_menu'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('INIT STATE: State Machine Has Started')
#        test_input = raw_input("test input: ")
	return 'to_menu'


# define state Bar
class Menu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        rospy.loginfo('MENU STATE: In Menu')
	yo = raw_input("insert test input: ")
        return 'outcome2'
        



# main
def main():
    rospy.init_node('mrscot3_state_machine')
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['End'])
    sm.userdata.sm_if_shutdown = 'n'

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INIT', Init(), 
                               transitions={'to_menu':'MENU'})
        smach.StateMachine.add('MENU', Menu(), 
                               transitions={'outcome2':'INIT'})
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
