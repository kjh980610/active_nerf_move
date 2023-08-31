import cv2
import numpy as np
import os
import glob

import pykin.utils.transform_utils as t_util
from pykin.kinematics.transform import Transform

# 체커보드의 차원 정의
CHECKERBOARD = (7,10) # 체커보드 행과 열당 내부 코너 수
INTER_CORNER_DISTANCE_MM = 0.025  # 체커보드 내부 코너 사이의 간격 (25mm)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# 각 체커보드 이미지에 대한 3D 점 벡터를 저장할 벡터 생성
objpoints = []
# 각 체커보드 이미지에 대한 2D 점 벡터를 저장할 벡터 생성
imgpoints = [] 

# 3D 점의 세계 좌표 정의
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * INTER_CORNER_DISTANCE_MM

prev_img_shape = None

# 주어진 디렉터리에 저장된 개별 이미지의 경로 추출
images = glob.glob('**/check_img/*.png')

for fname in images:
    img = cv2.imread(fname)
    # 그레이 스케일로 변환
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # 체커보드 코너 찾기
    # 이미지에서 원하는 개수의 코너가 발견되면 ret = true
    ret, corners = cv2.findChessboardCorners(gray,
                                             CHECKERBOARD,
                                             cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    # 원하는 개수의 코너가 감지되면,
    # 픽셀 좌표 미세조정 -> 체커보드 이미지 표시
    if ret == True:
        objpoints.append(objp)
        # 주어진 2D 점에 대한 픽셀 좌표 미세조정
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        imgpoints.append(corners2)
        # 코너 그리기 및 표시
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    cv2.imshow('img',img)
    print(fname)
    cv2.waitKey(0)
cv2.destroyAllWindows()
h,w = img.shape[:2] # 480, 640
# 알려진 3D 점(objpoints) 값과 감지된 코너의 해당 픽셀 좌표(imgpoints) 전달, 카메라 캘리브레이션 수행

# 카메라 내부 파라미터
camera_matrix = np.array([607.0380859375, 0.0, 313.9091796875, 0.0, 606.7507934570312, 256.9510803222656, 0.0, 0.0, 1.0])
camera_matrix = camera_matrix.reshape(3,3)

# 왜곡 계수
dist_coeffs = np.array([0,0,0,0,0])


vecs=[]
# 카메라 자세와 위치 추정
for i in range(len(objpoints)) :
    retval, rvec, tvec = cv2.solvePnP(objpoints[i], imgpoints[i], camera_matrix, dist_coeffs)
    angle = t_util.vector_norm(rvec)
    axis = rvec / angle
    rvec = t_util.get_quaternion_from_axis_angle(axis,angle)
    vec = Transform(pos=np.array([tvec[0][0],tvec[1][0],tvec[2][0]]),rot=np.array([rvec[0][0],rvec[1][0],rvec[2][0],rvec[3][0],]))
    vecs.append(vec)
    print('pnp'+str(i+1)+'=' + str(vec)+'.h_mat')

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

vecs=[]
print("\ncalib")
print(mtx)
for i in range(len(objpoints)):
    rvec=rvecs[i]
    tvec=tvecs[i]
    angle = t_util.vector_norm(rvec)
    axis = rvec / angle
    rvec = t_util.get_quaternion_from_axis_angle(axis,angle)
    vec = Transform(pos=np.array([tvec[0][0],tvec[1][0],tvec[2][0]]),rot=np.array([rvec[0][0],rvec[1][0],rvec[2][0],rvec[3][0],]))
    vecs.append(vec)
    print('pnp'+str(i+1)+'=' + str(vec)+'.h_mat')

