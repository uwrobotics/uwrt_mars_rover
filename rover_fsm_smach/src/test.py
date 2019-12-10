#!/usr/bin/env python
import roslib

# roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import time


# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'

class Forward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('going forward')
        time.sleep(1)
        return 'out2'


class Reverse(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):
        rospy.loginfo('backing up')
        time.sleep(1)
        return 'out1'


class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):
        rospy.loginfo('stopping motor')
        time.sleep(1)
        return 'out1'



def main():
    rospy.init_node('smach_test')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sis = smach_ros.IntrospectionServer('test', sm, '/SM_ROOT')
    sis.start()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'outcome1': 'Forward', 'outcome2': 'Reverse'})
        smach.StateMachine.add('Forward', Forward(),
                               transitions={'out2': 'Reverse'})
        smach.StateMachine.add('Reverse', Reverse(),
                               transitions={'out1': 'Stop'})
        smach.StateMachine.add('Stop', Stop(),
                               transitions={'out1': 'FOO'})
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
    print("done")


if __name__ == '__main__':
    main()