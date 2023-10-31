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

from pytamp.scene.scene_manager import SceneManager
from pytamp.planners.rrt_star_planner import RRTStarPlanner
from pytamp.planners.cartesian_planner import CartesianPlanner
from collections import OrderedDict

from move_test import go_joint_paths

import image_saver
import time
from get_pose import get_pose


def rotate_capture() :

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

    #rrt path 생성
    def update_rrt_path(scene_mngr, goal_pose,joint_path, goal_q = None):       
        planner = RRTStarPlanner(delta_distance=0.05, epsilon=0.2, gamma_RRT_star=2, dimension = robot.arm_dof)
        planner.run(scene_mngr=scene_mngr, cur_q=joint_path[-1], goal_pose=goal_pose, max_iter=500, goal_q=goal_q)
        joint_path += planner.get_joint_path(n_step=10)

    #cartesian path 생성
    def update_cartesian_path(scene_mngr,goal_pose,joint_path):        
        planner = CartesianPlanner(n_step = 2, dimension = robot.arm_dof)
        planner.run(scene_mngr=scene_mngr, cur_q=joint_path[-1], goal_pose=goal_pose)
        joint_path += planner.get_joint_path()
         

    #path 초기값
    joint_path =[init_qpos]         #[[qpos(7 joint angles)],...]
    joint_pathes = OrderedDict()    #OrderedDict([('task',[joint_path]),... ])
    pathes_num = 0                  #
    
    #로봇 초기 eef pose 저장
    init_pose = scene_mngr.get_robot_eef_pose()                     

    #회전 중심 위치 설정
    xi = init_pose[0,3]+0.05
    yi = init_pose[1,3]
    h = init_pose[2,3]

    #카메라(eef) 각도 설정
    th = math.radians(75)
    #회전 반경 설정
    r = 0.16

    #step 횟수 설정
    num = 9

    #eef pose 저장을 위한 배열 선언 -> eef to camera 변환 행렬 알면 camera pose 바로 얻을 수 있음
    t_poses = [None] * num

    #마지막 조인트 꼬임 방지를 위한 go default 체크용 bool
    go_def = True

    for i in range(1):
        dp = np.pi * 2 / num
        p = dp * i
        t_pose = Transform(pos=[xi-r*math.cos(p),yi-r*math.sin(p),h], rot=[0.001,-th-np.pi/2,p+np.pi])
        t_poses[i] = t_pose.h_mat

        # if joint_path[-1][-1] < dp/2-np.pi and go_def:
        #     print("go_def")
        #     update_rrt_path(scene_mngr, init_pose, joint_path, goal_q = init_qpos)
        #     joint_pathes.update({"go_def" : joint_path[pathes_num:]})
        #     pathes_num = len(joint_path)+1
        #     go_def = False

        step="#step" + str(i+1)
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
        joint_path = value    
        print(task + '\tpath length:' +str(len(joint_path)))
        go_joint_paths(joint_path)
        if 'step' in task:
            tp = get_pose(i+1)
            image_saver.ImageSaver(tp,i+1,check_board=True)
            i=i+1
        time.sleep(2)

rotate_capture()
