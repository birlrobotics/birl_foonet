 #! /usr/bin/env python
import rospy
import actionlib
import birl_foonet.msg
from collections import OrderedDict

def read_foo(file_name):
    
    _file = open(file_name, 'r')
    items = _file.read().splitlines()
    phase_count = 0
    object_id = None
    state_id = None
    object_info = []
    x = 0

    while x < len(items):
        line = items[x]
        if line.startswith("//"):
            phase_count += 1
        elif line.startswith("O"):
            objectParts = line.split("O")
            objectParts = objectParts[1].split("\t")
            object_id = int(objectParts[0])
            x += 1
            line = items[x]
            stateParts = line.split("S"); # get the Object's state identifier by splitting first instance of S
            stateParts = stateParts[1].split("\t");
            state_id = int(stateParts[0])
    
            state_dic = {"phase":phase_count,
                        "object_id":object_id,
                        "state_id":state_id,}
        object_info.append(state_dic) 
        x += 1
    obj_ids = []
    for dic in object_info:
        obj_ids.append(dic["object_id"])
    
    s = []
    for i in obj_ids:
        if i not in s:
            s.append(i)
    for id_type in s:
        if id_type = 1:
            object_ID = [0,1,14]

    return object_ID


def foonet_client(object_id):

    client = actionlib.SimpleActionClient("foonet_action", birl_foonet.msg.FoonetAction)     
    if client.wait_for_server(rospy.Duration(3.0)):
        goal = birl_foonet.msg.FoonetGoal(object_id)
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

if __name__ == '__main__':
    
    rospy.init_node('foonet_client_py')
    file_name = "../scripts/FOON_task_tree.txt"
    object_id = read_foo(file_name)
    result = foonet_client(object_id)
    print (result)
