#!/usr/bin/env python

import sys
import rospy as ros

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

def go_joint_paths(joint_paths):
    for joint_pose in joint_paths:
        go_to_joint_state(joint_pose)

def go_to_joint_state(joint_pose):
    ros.init_node('move_to_start')

    action = ros.resolve_name('effort_joint_trajectory_controller/follow_joint_trajectory')
    client = SimpleActionClient(action, FollowJointTrajectoryAction)
    # ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
    client.wait_for_server()

    pose = {'panda_joint1': 0, 
            'panda_joint2': -0.785398163397, 
            'panda_joint3': 0, 
            'panda_joint4': -2.35619449019, 
            'panda_joint5': 0, 
            'panda_joint6': 1.57079632679, 
            'panda_joint7': 0.785398163397}

    # poses = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]
    # poses = [0, 0.1963495375, 0.00, -2.616, 0.00, 2.9415926, 0.79]

    if joint_pose is not None :
        for i in range(7):  
            pose['panda_joint'+str(i+1)] = joint_pose[i]

    # ros.loginfo(pose)

    topic = ros.resolve_name('franka_state_controller/joint_states')
    # ros.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
    joint_state = ros.wait_for_message(topic, JointState)
    initial_pose = dict(zip(joint_state.name, joint_state.position))

    max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

    point = JointTrajectoryPoint()
    point.time_from_start = ros.Duration.from_sec(
        # Use either the time to move the furthest joint with 'max_dq' or 500ms,
        # whatever is greater
        max(max_movement / 0.2, 0.5)
    )
    goal = FollowJointTrajectoryGoal()

    goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
    point.velocities = [0] * len(pose)

    goal.trajectory.points.append(point)
    goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

    # ros.loginfo('Sending trajectory Goal to move into initial config')
    client.send_goal_and_wait(goal)

    result = client.get_result()
    if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
        ros.logerr('move_to_start: Movement was not successful: ' + {
            FollowJointTrajectoryResult.INVALID_GOAL:
            """
            The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
            Is the 'joint_pose' reachable?
            """,

            FollowJointTrajectoryResult.INVALID_JOINTS:
            """
            The joint pose you specified is for different joints than the joint trajectory controller
            is claiming. Does you 'joint_pose' include all 7 joints of the robot?
            """,

            FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
            """
            During the motion the robot deviated from the planned path too much. Is something blocking
            the robot?
            """,

            FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
            """
            After the motion the robot deviated from the desired goal pose too much. Probably the robot
            didn't reach the joint_pose properly
            """,
        }[result.error_code])

    else:
        # ros.loginfo('move_to_start: Successfully moved into pose')
        pass
