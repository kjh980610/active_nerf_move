import rospy
import tf

rospy.init_node('tf_listener')

# 프레임 이름 설정
frames = ['panda_hand_tcp', 'camera_link', 'panda_link0']

# TF 정보 수신을 위한 TransformListener 생성
tf_listener = tf.TransformListener()

time = rospy.Time()
for target_frame in frames:
    for source_frame in frames:
        if target_frame != source_frame:
            # TF 정보 수신 대기
            try:
                tf_listener.waitForTransform(target_frame, source_frame, time, rospy.Duration(4.0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(f"Transformation not available for {target_frame} -> {source_frame}")
                continue

            # 변환 정보 가져오기
            try:
                (position, orientation) = tf_listener.lookupTransform(target_frame, source_frame, time)
                print(f"Transform from {source_frame} to {target_frame}:")
                print(f"pos=[{position[0]}, {position[1]},{position[2]}],")
                print(f"rot=[{orientation[3]},{orientation[0]}, {orientation[1]}, {orientation[2]}]")
                print()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn(f"Failed to get transform from {source_frame} to {target_frame}")