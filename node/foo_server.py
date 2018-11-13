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
from foo_help_fnc import compute_transfom,table_pose


# global variables
marker_msg = None
robot_endpoint_msg = None
_rate = 100
flag_marker = False
flag_endpoint = False



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

        self.pub_result(goal)
        self._as.set_succeeded(self._result)    

        # while not rospy.is_shutdown():
        #     self.pub_feedback()  
        #     r.sleep()


    def pub_result(self, goal):
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



    # duo to the need of continously publishing feedback,so we have while.
    # feedback: transfrom bettwen object and robot gripper
    #           transform bettwen object and table(tale pose is set by human)
    #           transform bettwen object and human hand (To do)
    def pub_feedback(self):               
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
        self._feedback = birl_foonet.msg.FoonetFeedback() # re-init feedback


def select_image_prec(image_proc_method = "ar_marker"):           
# choose image procesing method
    if image_proc_method == "ar_marker":
        if rospy.wait_for_message("ar_pose_marker",AlvarMarkers,timeout=5): # wait for topic
            rospy.Subscriber("ar_pose_marker", AlvarMarkers, image_proc_cb)
        else:
            rospy.logfatal("Could not connect to /ar_pose_marker server, is the server node running?")
            rospy.signal_shutdown("Required component missing"); 
            
    elif image_proc_method == "baysian_filter":
        if rospy.wait_for_message("ar_pose_marker",AlvarMarkers,timeout=5):
            rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
        else:
            rospy.logfatal("Could not connect to /ar_pose_marker server, is the server node running?")
            rospy.signal_shutdown("Required component missing"); 

if __name__ == '__main__':
    rospy.init_node('foonet_server') # rosnode name 
    select_image_prec("ar_marker")
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, robot_endpoint_cb)
    server = FoonetClass()
    # rospy.spin() # keep code running 