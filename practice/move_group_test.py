#!/usr/bin/env python
# Python 2/3 compatibility imports

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import time

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterface"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

        robot = moveit_commander.RobotCommander()
        # print(robot.get_current_state())

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()

        # We can also print the name of the end-effector link for this group:
        move_group.set_end_effector_link("panda_hand")
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()


        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    
    def go_to_joint_state(self):

        move_group = self.move_group


        joint_goal = move_group.get_current_joint_values()
        print(joint_goal)
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self,pose_goal):

        move_group = self.move_group

        move_group.set_pose_target(pose_goal,self.eef_link)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_and_execute_cartesian_path(self, pose_goal):

        move_group = self.move_group

        waypoints = []
        waypoints.append(pose_goal)

        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  
    
        move_group.execute(plan, wait=True)

        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):

        move_group = self.move_group

        move_group.execute(plan, wait=True)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))


        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):

        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names


        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):

        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name=box_name)
        
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

import franka_gripper.msg
import actionlib

def grasp_client(open=True):
    # Creates the SimpleActionClient, passing the type of the action
    # (GraspAction) to the constructor.
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.GraspGoal()

    if open:
        goal.width=0.08
        goal.speed = 0.1

    else :
        goal.width = 0.03
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.005
        goal.speed = 0.1
        goal.force = 10

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GraspResult

def main():
    try:
        tutorial = MoveGroupPythonInterface()

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        # tutorial.go_to_joint_state()
        # time.sleep(1)

        print("============a movement using a pose goal ...")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 1
        pose_goal.position.x = 0.45
        pose_goal.position.y = 0
        pose_goal.position.z = 0.4

        tutorial.go_to_pose_goal(pose_goal)



        pose_goal.position.x = 0.4
        pose_goal.position.z = 0.59

        print("============ Press `Enter` to execute a saved path ...")
        tutorial.plan_and_execute_cartesian_path(pose_goal)

        # print("============ Press `Enter` to add a box to the planning scene ...")
        # tutorial.add_box()
        # time.sleep(1)


        # print("============ Press `Enter` to attach a Box to the Panda robot ...")
        # tutorial.attach_box()
        # time.sleep(1)

        # # print(
        # #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # # )
        # # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # # tutorial.execute_plan(cartesian_plan)

        # print("============ Press `Enter` to detach the box from the Panda robot ...")
        # tutorial.detach_box()
        # time.sleep(1)

        # print(
        #     "============ Press `Enter` to remove the box from the planning scene ..."
        # )
        # tutorial.remove_box()
        # time.sleep(1)

        print("============ Python tutorial demo complete!")        

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()