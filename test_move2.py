#!/usr/bin/env python
# Python 2/3 compatibility imports

import control
import rospy
import math
from pykin.kinematics.transform import Transform
import pykin.utils.transform_utils as tu
import numpy as np


def main(): 
    try:
        move_group= control.control()

        # move_group.go_to_start()

        object_position = [0.5,0,0.03] #사진 찍을 물체 위치(현재 aruco marker position)

        angle = math.pi/6
        d= 0.3

        print(math.cos(angle)*d) 
        print(math.sin(angle)*d)

        panda_hand_to_camera_color_frame = Transform(
            pos=[0.04147810144921763, -0.04429576589937319,0.03795743837848484],
            rot=[0.7009038903068452,-0.0017877762181792646, 0.02117626918199301, 0.7129390658703675]).h_mat
        
        t_pos = [object_position[0]-math.sin(angle)*d,object_position[1],object_position[2]+math.cos(angle)*d]

        
        cam_pose = Transform(pos=t_pos, rot=[-math.pi+angle, 0, -math.pi/2])
        # cam_pose = Transform(pos=[0.44137, 0.044371, 0.41186], rot=[-math.pi, 0, -math.pi/2])

        goal_pose = np.dot(cam_pose.h_mat, tu.get_inverse_homogeneous(panda_hand_to_camera_color_frame))
        goal_pose = tu.get_pose_from_homogeneous(goal_pose)

        pose = move_group.cal_target_pose(pos = goal_pose[0:3], rot=goal_pose[3:] )
        move_group.go_to_pose_goal(pose)


        # move_group.go_to_start()



    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()