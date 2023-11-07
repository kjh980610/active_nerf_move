import numpy as np
import geometry_msgs.msg
import rospy
import tf
from tf.transformations import quaternion_from_matrix
from tf.transformations import rotation_matrix
from tf.transformations import translation_from_matrix
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_matrix

def spherical_to_cartesian(r, theta, phi):
    
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return np.array([x, y, z])

def vector_to_rotation_matrix(vector, theta):
    vector = vector / np.linalg.norm(vector)

    I = np.eye(3)
    
    K = np.array([[0.0, -vector[2], vector[1]],
                  [vector[2], 0.0, -vector[0]],
                  [-vector[1], vector[0], 0.0]])
    
    # Rodrigues' formula
    R = I + (np.sin(theta) * K) + (1 - np.cos(theta)) * np.matmul(K, K)
    return R

def generate_cam_pose(r, theta, phi):
    point = spherical_to_cartesian(r, theta, phi)
    x, y, z = point
    axis = np.array([0, 0, -1.])
    vector = np.stack([-x, -y, -z])
    
    rotation_axis = np.cross(point, axis)
    cos_theta = np.dot(-point, axis) / (np.linalg.norm(point) * np.linalg.norm(axis))
    rotation_theta = np.arccos(cos_theta)
    
    rot_mat = vector_to_rotation_matrix(rotation_axis, rotation_theta)
    tf_mat = np.hstack([rot_mat, np.reshape(point, (3, 1))])
    tf_mat = np.vstack([tf_mat, [0.0, 0.0, 0.0, 1.0]])
    return tf_mat

def generate_eef_pose(tf_elem):
    cam_trans, cam_quat = tf_elem

    x_axis_flip = np.array([1.0, 0.0, 0.0, 0.0])
    cam_quat = quaternion_multiply(cam_quat, x_axis_flip)
    
    target_frame = "0_pose"
    source_frame = "panda_link0"
    listener = tf.TransformListener()
    listener.waitForTransform(target_frame, source_frame,
                              rospy.Time(), rospy.Duration(4.0))
    
    (base_to_cam_trans, base_to_cam_quat) = listener.lookupTransform(target_frame,
                                            source_frame,
                                            rospy.Time(0))
    
    base_to_cam_mat = quaternion_matrix()
    


if __name__ == "__main__":

    tf_dict = {}
    num = 5
    for i in range(num):
        i_theta = np.random.uniform(-np.pi/3, np.pi/3)
        i_phi = np.random.uniform(0.0, np.pi/4)

        tf_mat = generate_cam_pose(r=0.54, theta=i_theta, phi=i_phi)
        trans = translation_from_matrix(tf_mat)
        quat = quaternion_from_matrix(tf_mat)
        x_axis_flip = np.array([1.0, 0.0, 0.0, 0.0]) # xyzw
        quat = quaternion_multiply(quat, x_axis_flip)

        tf_dict[f'{i}_pose'] = [trans, quat]

    rospy.init_node("testt")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    # generate_eef_pose(tf_dict['0_pose'])

    print(tf_dict['0_pose'][0])
    print(tf_dict['1_pose'][1])

    while not rospy.is_shutdown():
        for i in range(num):
            name = f"{i}_pose"
            
            br.sendTransform(tf_dict[f'{i}_pose'][0],
                             tf_dict[f'{i}_pose'][1],
                             rospy.Time.now(),
                             name,
                             "aruco_marker_frame")
