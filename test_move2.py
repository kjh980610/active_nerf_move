#!/usr/bin/env python
# Python 2/3 compatibility imports

import control
import rospy
import math
from pykin.kinematics.transform import Transform
import pykin.utils.transform_utils as tu


def main(): 
    try:
        move_group= control.control()

        move_group.go_to_start()

        object_position = [0.5,0,0.03] #사진 찍을 물체 위치(현재 aruco marker position)

        cam_dist = 0.2
        angle = math.pi/6

        

        pose = move_group.cal_target_pose(pos = [0.4,0,0.59],rot=[0.0, 0.96593, -0.0, 0.25882] )
        move_group.plan_and_execute_cartesian_path(pose)


        # move_group.go_to_start()



    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()