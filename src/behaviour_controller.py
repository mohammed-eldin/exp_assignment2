#!/usr/bin/env python

## @package behaviour_controller
#
# state machine to control the behaviour of the pet
# States: NORMAL, SLEEP, PLAY

import rospy
import smach
import smach_ros
import random
from std_msgs.msg import String
from std_msgs.msg import Bool


## current behaviour publisher
pub_state = rospy.Publisher("/behaviour",String,queue_size=1)

sub_home = None

## class state Normal
#
# normal behaviour of the pet
class Normal(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['go_to_sleep','go_play']
                            )
        
        self.ball_detected = False
        self.rate = rospy.Rate(20)  # Loop at 100Hz

    ## method execute
    #
    # state execution
    def execute(self, userdata):
        rospy.loginfo('Executing state NORMAL')
        pub_state.publish("normal")

        ## check if a voice command is received
        rospy.Subscriber("/ball_detected", Bool, self.get_ball_detection)
        count = 0

        init_time = rospy.Time.now()

        while not rospy.is_shutdown():  
            # count time passed from the start of Normal state
            if count == 1:
                init_time = rospy.Time.now()
            count = count + 1
            current_time = rospy.Time.now()
            time_passed = current_time.secs - init_time.secs

            if (self.ball_detected):
                ## If the robot sees the ball goes to the play behaviour
                return 'go_play' 
                    
            elif (random.randint(1,1000) == 1 and time_passed > 30):
                ## go to sleep at random 
                #  (1/10000 chances per iteration -> 100 iterations per second -> 1/100 chance per second passed in Normal state)
                return 'go_to_sleep'
                    
            self.rate.sleep()
    
    ## method get_ball_detection
    #
    # subscriber callback for ball detection
    def get_ball_detection(self, ball):
        self.ball_detected = ball.data
            


## class state Sleep
#
# Sleep behaviour of the pet
class Sleep(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['wake_up']
                            )
        self.home_reached = False
        self.rate = rospy.Rate(20)
        
    ## method execute
    #
    # state execution
    def execute(self, userdata):
        rospy.loginfo('Executing state SLEEP')
        pub_state.publish("sleep")

        # home position reached subscriber
        sub_home = rospy.Subscriber("/home_reached", Bool, self.get_home_reached)
        
        while not rospy.is_shutdown():  
            # check if the robot is in home position
            if(self.home_reached):
                ## wait some time to wake up
                rospy.sleep(random.randint(20,40))
                self.home_reached = False
                return 'wake_up'
            self.rate.sleep
        
    ## method get_home_reached
    #
    # subscriber callback, gets if the robot is in the home position
    def get_home_reached(self,home_reached):    
        self.home_reached = home_reached.data
    

## class state Play
#
# Play behaviour of the pet
class Play(smach.State):
    ## method init
    #
    # state initialization
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['stop_play'],
                            )
        self.ball_detected = False
        self.rate = rospy.Rate(1)
        self.counter = 0

    ## method execute
    #
    # state execution
    def execute(self,userdata):
        rospy.loginfo('Executing state PLAY')
        pub_state.publish("play")
        
        ## subscriber to ball detection
        rospy.Subscriber("/ball_detected", Bool, self.get_ball_detection)

        while not rospy.is_shutdown():  
            # check if the ball is not detected
            if(not self.ball_detected):
                self.counter = self.counter + 1
                rospy.loginfo("Searching for the ball... "+str(self.counter)+" seconds")
                # if the ball is not detected for 10 seconds straight
                if self.counter > 15:
                    return 'stop_play'
            elif(self.ball_detected):
                self.counter = 0

            # loop every 1 second
            self.rate.sleep()

    ## method get_ball_detection
    #
    # subscriber callback for ball detection
    def get_ball_detection(self, ball):
        self.ball_detected = ball.data
        
    
## function main 
#
# Initiaize the state machine
def main():
    rospy.init_node("behaviour_controller")

    # wait for the initialization of the system
    rospy.sleep(2)

    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    ## Open the container
    with sm:
        ## Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_to_sleep':'SLEEP', 
                                            'go_play':'PLAY'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wake_up':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'stop_play':'NORMAL'})

    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute the state machine
    outcome = sm.execute()

    ## Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == "__main__":
    main()

