#!/usr/bin/env python

## @package human_interaction_gen
#
# Human interactions with the ball: makes the ball move or go underground

import rospy
import random
from geometry_msgs.msg import PoseStamped
import actionlib
import actionlib.msg
from exp_assignment2.msg import PlanningAction, PlanningActionGoal

act_c = None
goal_pos = PlanningActionGoal()

## function get_random_position
#
# get a random position on the map for the ball
def get_random_position():
    randX = random.randint(-8, 8)
    randY = random.randint(-8, 8)
    randZ = 0.5
    randPos = [randX, randY, randZ]
    return randPos

## function move_ball
#
# move the ball in a random position on the map
def move_ball():
    # get a random position
    pos = get_random_position()

    # set ball goal position 
    goal_pos.goal.target_pose.pose.position.x = pos[0]
    goal_pos.goal.target_pose.pose.position.y = pos[1]
    goal_pos.goal.target_pose.pose.position.z = pos[2]

    # send ball position and wait that the goal is reached within 60 seconds
    act_c.send_goal(goal_pos.goal)
    act_c.wait_for_result(rospy.Duration.from_sec(60.0))


## function ball_disappear
#
# make the ball disappear under the map
def ball_disappear():
    # get a random position
    pos = get_random_position()

    # set ball goal position under the map
    goal_pos.goal.target_pose.pose.position.x = 0
    goal_pos.goal.target_pose.pose.position.y = 0
    goal_pos.goal.target_pose.pose.position.z = -1

    # send ball position and wait that the goal is reached within 60 seconds
    act_c.send_goal(goal_pos.goal)
    act_c.wait_for_result(rospy.Duration.from_sec(60.0))
    
## function main
#
# initialize node, action client and makes the ball move on the map or underground
def main():
    # init node
    rospy.init_node("human_interaction_generator")
    rate = rospy.Rate(20)
    global act_c

    # initialize action client
    act_c = actionlib.SimpleActionClient('/ball/reaching_goal', PlanningAction)
    # wait for the initialization of the server for five seconds
    act_c.wait_for_server(rospy.Duration(5))

    while not rospy.is_shutdown():
        # random choice 
        if random.randint(1,4) == 1:
            # make the ball disappear
            ball_disappear()
        else:
            # move the ball
            move_ball()
        # wait random time
        rospy.sleep(random.randint(7,10))

        rate.sleep()


if __name__ == "__main__":
    main()
