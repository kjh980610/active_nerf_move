#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

import os
import yaml

from pykin import assets
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh
from pykin.utils.transform_utils import get_rpy_from_matrix
from pykin.utils import plot_utils as p_utils

from pytamp.scene.scene_manager import SceneManager
from pytamp.planners.rrt_star_planner import RRTStarPlanner
from pytamp.planners.cartesian_planner import CartesianPlanner
from collections import OrderedDict

from move_test import go_joint_paths

from numpy import array

import image_saver
import time


def target_rotate() :

    current_pose = Transform(pos=np.array([0,0,0]), rot=np.array([0.0, 0.0, 0.0]))
    rotate_pose = Transform(pos=np.array([0,0,0]), rot=np.array([0.0, 0.0, np.pi/2]))

    asset_file_path = os.path.abspath(assets.__file__ + "/../")

    # 로봇 불러오기
    file_path = "urdf/panda/panda.urdf"
    robot = SingleArm(
        f_name=file_path,
        offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
        has_gripper=True,
        #gripper_name="robotiq140",
    )

    robot.setup_link_name("panda_link_0", "right_hand")

    custom_fpath = asset_file_path + "/config/panda_init_params.yaml"
    with open(custom_fpath) as f:
        controller_config = yaml.safe_load(f)
    init_qpos = controller_config["init_qpos"]
    init_qpos = [0, -0.785398163397, 0, -2.35619449019, 0, 1.57079632679, 0.785398163397]


    #로봇 초기 설정
    scene_mngr = SceneManager("collision", is_pyplot=True)
    scene_mngr.add_robot(robot, init_qpos)


    # 테이블 씬 설정
    table_mesh = get_object_mesh("ben_table.stl",  scale=[1.0, 1.5, 1.0])
    table_height = table_mesh.bounds[1][2] - table_mesh.bounds[0][2]
    table_pose = Transform(
                pos=np.array([1.0, -0.6, table_mesh.bounds[0][2]])
            )
    scene_mngr.add_object(
        name="table", gtype="mesh", gparam=table_mesh, h_mat=table_pose.h_mat, color=[0.823, 0.71, 0.55]
    )


    #타겟 씬 설정
    red_cube_mesh = get_object_mesh("ben_cube.stl",scale=[0.15, 0.1, 0.1])
    target_offset = np.array([0.6, 0, table_height + abs(red_cube_mesh.bounds[0][2])+0.073])
    current_pose = Transform(pos = current_pose.pos + target_offset, rot=current_pose.rot)
    rotate_pose = Transform(pos = rotate_pose.pos + target_offset, rot=rotate_pose.rot)

    scene_mngr.add_object(
        name="target",
        gtype="mesh",
        gparam=red_cube_mesh,
        h_mat=current_pose.h_mat,
        color=[1.0, 0.0, 0.0],
    )


    def update_rrt_path(scene_mngr, goal_pose,joint_path, goal_q = None):        #rrt로 path 생성
        planner = RRTStarPlanner(delta_distance=0.05, epsilon=0.2, gamma_RRT_star=2, dimension = robot.arm_dof)
        planner.run(scene_mngr=scene_mngr, cur_q=joint_path[-1], goal_pose=goal_pose, max_iter=500, goal_q=goal_q)
        joint_path += planner.get_joint_path(n_step=1)

    def update_cartesian_path(scene_mngr,goal_pose,joint_path):        #cartesian path 생성
        planner = CartesianPlanner(n_step = 5, dimension = robot.arm_dof)
        planner.run(scene_mngr=scene_mngr, cur_q=joint_path[-1], goal_pose=goal_pose)
        joint_path += planner.get_joint_path()
         

    #path 초기값
    joint_path =[init_qpos]         #[[qpos(7 joint angles)],...]
    joint_pathes = OrderedDict()    #OrderedDict([('task',[joint_path]),... ])
    pathes_num = 0                  #

    init_pose = scene_mngr.get_robot_eef_pose()                     #로봇 초기 eef pose 저장

    # poses = [get_rpy_from_matrix(init_pose)]

    xi = init_pose[0,3]+0.05
    yi = init_pose[1,3]

    th = math.radians(75)
    h = init_pose[2,3]
    r = 0.15

    p = np.pi/2
    num = 4
    t_poses = [None] * num

    first_pose = Transform(pos=[xi,yi,h], rot = [0.001,np.pi,0])
    first_pose = first_pose.h_mat

    update_cartesian_path(scene_mngr, first_pose, joint_path)
    joint_pathes.update({"init" : joint_path[pathes_num:]})
    pathes_num = len(joint_path)+1
    

    for i in range(1):
        p = np.pi * (i)/num *2
        t_pose = Transform(pos=[xi-r*math.cos(p),yi-r*math.sin(p),h], rot=[0.001,-th-np.pi/2,p+np.pi])
        t_poses[i] = t_pose.h_mat

        step="step" + str(i+1)
        print(step)
        update_rrt_path(scene_mngr, t_poses[i], joint_path)
        joint_pathes.update({step : joint_path[pathes_num:]})
        pathes_num = len(joint_path)+1

    #back_init_pose - 초기 위치로 돌아가기
    update_rrt_path(scene_mngr, init_pose, joint_path, goal_q = init_qpos)
    joint_pathes.update({"back" : joint_path[pathes_num:]})
    pathes_num = len(joint_path)+1

    i=0
    for task, value in joint_pathes.items():
        print(task)
        joint_path = value
        go_joint_paths(joint_path)
        if task != "back" and task != "init":
            image_saver.ImageSaver(t_poses[i],check_board=True)
            i=i+1
            time.sleep(1)


target_rotate()
