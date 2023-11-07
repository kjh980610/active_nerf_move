#!/usr/bin/env python
# Python 2/3 compatibility imports

import control
import rospy

def main():
    try:
        test= control.control()

        test.go_to_start()

        # print("============a movement using a pose goal ...")
        # pose_goal = test.cal_target_pose(pos=[0.4,0,0.35])
        # test.go_to_pose_goal(pose_goal)

        # pose_goal = test.cal_target_pose(pos=[0.4,0,0.22])
        # test.plan_and_execute_cartesian_path(pose_goal)
        # # test.grasp()

        # pose_goal = test.cal_target_pose(pos=[0.4,0,0.4])
        # test.plan_and_execute_cartesian_path(pose_goal)

        # pose_goal = test.cal_target_pose(pos=[0.4,0,0.4],rot=[0.00022829,0.91651,-0.40002, -0.00070195])
        # test.plan_and_execute_cartesian_path(pose_goal)

        # pose_goal = test.cal_target_pose(pos=[0.4,0,0.22],rot=[0.00022829,0.91651,-0.40002, -0.00070195])
        # test.plan_and_execute_cartesian_path(pose_goal)
        # # test.release()

        # pose_goal = test.cal_target_pose(pos=[0.4,0,0.4],rot=[0.00022829,0.91651,-0.40002, -0.00070195])
        # test.plan_and_execute_cartesian_path(pose_goal)


        # test.go_to_start()



    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()