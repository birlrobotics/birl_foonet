#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Pour
class Pour(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Pour')
        if self.counter < 2:
            self.counter += 1 
            return 'outcome1'
        else:
            return 'outcome2'   


# define state Crack
class Crack(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3',])

    def execute(self, userdata):
        rospy.loginfo('Executing state Crack')
        return 'outcome3'

# define state Mix
class Mix(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished',])

    def execute(self, userdata):
        rospy.loginfo('Executing state Mix')
        return 'finished'


# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Done'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(Pour.__name__, Pour(), 
                               transitions={'outcome1':'Crack', 
                                            'outcome2':'Mix'})

        smach.StateMachine.add(Crack.__name__, Crack(), 
                               transitions={'outcome3':'Pour'})

        smach.StateMachine.add(Mix.__name__, Mix(), 
                               transitions={'finished':'Done'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()