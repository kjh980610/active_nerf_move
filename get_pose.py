import rospy
import tf

rospy.init_node('tf_listener')

# 프레임 이름 설정
target_frame = 'world'
source_frame = 'fr3_hand_tcp'
source_frame2 = 'camera_link'

# TF 정보 수신을 위한 TransformListener 생성
tf_listener = tf.TransformListener()

# TF 정보 수신 대기
tf_listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))

# 변환 정보 가져오기
try:
    (position, orientation) = tf_listener.lookupTransform(target_frame, source_frame, rospy.Time())
    (position2, orientation2) = tf_listener.lookupTransform(target_frame, source_frame2, rospy.Time())
    (position3, orientation3) = tf_listener.lookupTransform(source_frame, source_frame2, rospy.Time())
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    rospy.logwarn("Transformation not available")

# rospy.loginfo(f"Position: x={position[0]}, y={position[1]}, z={position[2]}")
# rospy.loginfo(f"Orientation: x={orientation[0]}, y={orientation[1]}, z={orientation[2]}, w={rientation[3]}")

print(f"pos=[{position[0]}, {position[1]},{position[2]}],")
print(f"rot=[{orientation[0]}, {orientation[1]}, {orientation[2]}, {orientation[3]}]")

print(f"pos=[{position2[0]}, {position2[1]},{position2[2]}],")
print(f"rot=[{orientation2[0]}, {orientation2[1]}, {orientation2[2]}, {orientation2[3]}]")

print(f"pos=[{position3[0]}, {position3[1]},{position3[2]}],")
print(f"rot=[{orientation3[0]}, {orientation3[1]}, {orientation3[2]}, {orientation3[3]}]")
