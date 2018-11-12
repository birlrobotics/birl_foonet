#! /usr/bin/env python
import rospy
from geometry_msgs.msg import (Pose,PoseStamped)
from birl_foonet.srv import (
    Intereset_pose,
    Intereset_poseRequest,
    Intereset_poseResponse,
)
from birl_foonet.msg import foonet
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
import threading
import copy
import numpy
import ipdb


shared_msg = None

def object_anomaly_detetcion(pose):
    pass

def cb(msg):
    global shared_msg
    shared_msg = msg.markers

def call_back(req):
    global shared_msg
    # ipdb.set_trace()
    request_id = copy.deepcopy(req.foonet_request)
    rsp = Intereset_poseResponse()
    env_id = []
    found_id = []
    missing_id = []
    pose_list = []

    foonet_ = foonet()

    for msg in shared_msg:
        env_id.append(msg.id)

    for idx in request_id:
        if idx in env_id:
            found_id.append(idx)
        else:
            missing_id.append(idx)

    rsp.missing_id = missing_id
   
    for idx in found_id:
        for msg in shared_msg:
            if msg.id == idx:
                foonet_.object_id = idx
                foonet_.pose_list.pose = msg.pose.pose
                foonet_.pose_list.header = msg.header
                pose_list.append(foonet_)

    rsp.object_list = pose_list
    return rsp

def main():
    rospy.init_node("get_request_obj_id")
    rospy.wait_for_message("ar_pose_marker",AlvarMarkers)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
    s = rospy.Service("get_request_obj_id",Intereset_pose,call_back)
    rospy.spin()
if __name__=="__main__":
    main()