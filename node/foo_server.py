#! /usr/bin/env python

import rospy
import actionlib
import numpy as np
from copy import deepcopy
# normal libraries
import birl_foonet.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose,TransformStamped,Transform
from baxter_core_msgs.msg import EndpointState
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import ipdb


# global variables
marker_msg = None
robot_endpoint_msg = None
_rate = 1
flag_marker = False
flag_endpoint = False
image_processing_method = "ar_marker"

table_pose = np.array(
        ((1.0, 0.0, 0.0, -0.0),
        (0.0, 1.0, 0.0, -0.0),
        (0.0, 0.0, 1.0, 0.05),
        (0.0, 0.0, 0.0, 1.0))
    , dtype=np.float64)


def image_proc_cb(msg):
    global marker_msg,flag_marker
    if not flag_marker:  # check if we receive data
        print "maker signal is ok"
        flag_marker = True
    marker_msg = msg.markers

def robot_endpoint_cb(data):
    global flag_endpoint,robot_endpoint_msg
    if not flag_endpoint:
        flag_endpoint = True
        print 'endpoint state signals is OK!'
    robot_endpoint_msg = data.pose

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

class FoonetClass(object):
    # create messages that are used to publish feedback/result
    _feedback = birl_foonet.msg.FoonetFeedback()
    _result = birl_foonet.msg.FoonetResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer("foonet_action", birl_foonet.msg.FoonetAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # chech if goal is actived 
        if self._as.is_active():
            print "action server activated"     
       
        rospy.sleep(1) # sleep 1 s for waiting cb to get data

        r = rospy.Rate(_rate) # publish rate

        # check the objects available in the  environment
        env_id = []
        for msg in marker_msg: 
            env_id.append(msg.id)

        # compare the goal_id and the env_id, if goal_id is in the env_id, then add
        # to found_id, if not, then add to missing id. finally send as result to client 
        for _obj_idx in goal.object_id:
            if _obj_idx in env_id:  
                if  _obj_idx not in self._result.found_id:
                    self._result.found_id.append(_obj_idx)
            else:
                if _obj_idx not in self._result.missing_id:
                    self._result.missing_id.append(_obj_idx)
        self._result.found_id = sorted(self._result.found_id)
        self._result.missing_id = sorted(self._result.missing_id)
        self._as.set_succeeded(self._result)    

        # duo to the need of continously publishing feedback,so we have while.
        # feedback: transfrom bettwen object and robot gripper
        #           transform bettwen object and table(tale pose is set by human)
        #           transform bettwen object and human hand (To do)
        # ipdb.set_trace()

        while not rospy.is_shutdown():
            for _found_id in self._result.found_id:
                for msg in marker_msg:
                    if _found_id == msg.id:
                        tem_dic = compute_transfom(msg.pose.pose,robot_endpoint_msg,table_pose)
                        T_object_gripper = TransformStamped()
                        T_object_gripper.header = msg.header  
                        T_object_gripper.transform = tem_dic["marker_gripper"]
                        T_object_gripper.child_frame_id = str(msg.id)

                        T_object_table = TransformStamped()
                        T_object_table.transform = tem_dic["marker_table"]
                        T_object_table.child_frame_id = str(msg.id)
                        self._feedback.object_gripper_transfrom.append(T_object_gripper)
                        self._feedback.object_table_transfrom.append(T_object_table)                        
            self._as.publish_feedback(self._feedback)

            r.sleep()
                

            

if __name__ == '__main__':
    rospy.init_node('foonet_server') # rosnode name 
   
    # choose image procesing method
    if image_processing_method == "ar_marker":
        if rospy.wait_for_message("ar_pose_marker",AlvarMarkers,timeout=5): # wait for topic
            rospy.Subscriber("ar_pose_marker", AlvarMarkers, image_proc_cb)
        else:
            rospy.logfatal("Could not connect to /ar_pose_marker server, is the server node running?")
            rospy.signal_shutdown("Required component missing"); 
            
    elif image_processing_method == "baysian_filter":
        if rospy.wait_for_message("ar_pose_marker",AlvarMarkers,timeout=5):
            rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
        else:
            rospy.logfatal("Could not connect to /ar_pose_marker server, is the server node running?")
            rospy.signal_shutdown("Required component missing"); 
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, robot_endpoint_cb)
    server = FoonetClass()
    rospy.spin() # keep codo running 