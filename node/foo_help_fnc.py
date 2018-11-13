from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import numpy as np
from geometry_msgs.msg import Pose,TransformStamped,Transform

# do transform bettwen objects and other things
def compute_transfom(object_pose,gripper_pose,table_pose):
    # object pose object matrix
    object_mat = np.dot(translation_matrix((object_pose.position.x, object_pose.position.y, object_pose.position.z)), 
    quaternion_matrix((object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z,object_pose.orientation.w)))
    
    # robot endpoint pose to robot endpoint matrix
    endpoint_mat = np.dot(translation_matrix((gripper_pose.position.x, gripper_pose.position.y, gripper_pose.position.z)), 
    quaternion_matrix((gripper_pose.orientation.x, gripper_pose.orientation.y, gripper_pose.orientation.z,gripper_pose.orientation.w)))

    # transform bettwen endpoint and object
    T_gripper_marker = np.dot(np.linalg.inv(endpoint_mat),object_mat)
    # transform bettwen table and object
    T_table_marker = np.dot(np.linalg.inv(table_pose),object_mat)

    # change matrix to pose
    marker_gripper_trans = translation_from_matrix(T_gripper_marker)
    marker_gripper_quat = quaternion_from_matrix(T_gripper_marker)

    marker_gripper_tranform = Transform()
    marker_gripper_tranform.translation.x = marker_gripper_trans[0]
    marker_gripper_tranform.translation.y = marker_gripper_trans[1]
    marker_gripper_tranform.translation.z = marker_gripper_trans[2]
    marker_gripper_tranform.rotation.x = marker_gripper_quat[0]
    marker_gripper_tranform.rotation.y = marker_gripper_quat[1]
    marker_gripper_tranform.rotation.z = marker_gripper_quat[2]
    marker_gripper_tranform.rotation.w = marker_gripper_quat[3]

    marker_table_trans = translation_from_matrix(T_table_marker)
    marker_table_quat = quaternion_from_matrix(T_table_marker)

    marker_table_tranform = Transform()
    marker_table_tranform.translation.x = marker_table_trans[0]
    marker_table_tranform.translation.y = marker_table_trans[1]
    marker_table_tranform.translation.z = marker_table_trans[2]
    marker_table_tranform.rotation.x = marker_table_quat[0]
    marker_table_tranform.rotation.y = marker_table_quat[1]
    marker_table_tranform.rotation.z = marker_table_quat[2]
    marker_table_tranform.rotation.w = marker_table_quat[3]

    dic = {"marker_gripper":marker_gripper_tranform,
           "marker_table":marker_table_tranform, }
    return dic


table_pose = np.array(
        ((1.0, 0.0, 0.0, -0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.05),
        (0.0, 0.0, 0.0, 1.0))
    , dtype=np.float64)