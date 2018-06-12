import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetWorldProperties
from geometry_msgs.msg import *

import random
import numpy as np
import re

LIMIT = {'x':(-0.3, 0.5), 'y':(0.6, 1.4), 'rad':(0, 3.14)}
# QUATER = tf.transformations.quaternion_from_euler(0,0,0)
# ORIENT = Quaternion(QUATER[0], QUATER[1], QUATER[2], QUATER[3])

# We can't use scrap1 (due to gazebo7 issue) and tape3 (due to `.obj` issue),
# which is in the siemens code. Also Zisu didn't use screwdriver4.
MODEL_PATH = "/home/daniel/FETCH_CORE/examples/toolbox/"
MODEL_LIST = ["screwdriver1", "screwdriver2", "screwdriver3", "tape2", "tube1"]
MODEL_TYPE = ["screwdriver", "tape", "tube"]


def setup_delete_spawn_service():
    print("Waiting for gazebo services...")
    # rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    rospy.wait_for_service("gazebo/get_world_properties")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    object_monitor = rospy.ServiceProxy("gazebo/get_world_properties", GetWorldProperties)

    return delete_model, spawn_model, object_monitor


def get_object_list(object_monitor):
    lst = []
    for name in object_monitor().model_names:
        ind = re.search("\d", name)
        if ind == None:
            continue
        else:
            ind = ind.start()

        tag = name[:ind]
        if tag in MODEL_TYPE:
            lst.append(name)
    return lst


def delete_object(name, delete_model):
    delete_model(name)


def clean_floor(delete_model, object_monitor):
    object_lst = get_object_list(object_monitor)
    for obj in object_lst:
        delete_object(obj, delete_model)
        rospy.sleep(0.5)


def spawn_from_uniform(n, spawn_model):
    for i in range(n):
        # item
        model_tag = random.choice(MODEL_LIST)

        with open(MODEL_PATH+model_tag+"/model.sdf", "r") as f:
            object_xml = f.read()

        # pose
        pt_x = np.random.uniform(LIMIT['x'][0], LIMIT['x'][1])
        pt_y = np.random.uniform(LIMIT['y'][0], LIMIT['y'][1])
        ei = np.random.uniform(LIMIT['rad'][0], LIMIT['rad'][1])
        ej = np.random.uniform(LIMIT['rad'][0], LIMIT['rad'][1])
        ek = np.random.uniform(LIMIT['rad'][0], LIMIT['rad'][1])
        quater = tf.transformations.quaternion_from_euler(ei, ej, ek)
        orient = Quaternion(quater[0], quater[1], quater[2], quater[3])

        object_pose = Pose(Point(x=pt_x, y=pt_y, z=0.5), orient)

        # spawn
        object_name = model_tag+"_"+str(i)
        spawn_model(object_name, object_xml, "", object_pose, "world")
        rospy.sleep(0.5)


def spawn_from_gaussian(n, spawn_model):
    for i in range(n):
        # item
        model_tag = random.choice(MODEL_LIST)

        with open(MODEL_PATH+model_tag+"/model.sdf", "r") as f:
            object_xml = f.read()

        # pose
        pt_x = np.random.normal((LIMIT['x'][0]+LIMIT['x'][1])/2, (LIMIT['x'][1]-LIMIT['x'][0])/8)
        pt_y = np.random.normal((LIMIT['y'][0]+LIMIT['y'][1])/2, (LIMIT['y'][1]-LIMIT['y'][0])/8)
        ei = np.random.normal((LIMIT['rad'][0]+LIMIT['rad'][1])/2, (LIMIT['rad'][1]-LIMIT['rad'][0])/8)
        ej = np.random.normal((LIMIT['rad'][0]+LIMIT['rad'][1])/2, (LIMIT['rad'][1]-LIMIT['rad'][0])/8)
        ek = np.random.normal((LIMIT['rad'][0]+LIMIT['rad'][1])/2, (LIMIT['rad'][1]-LIMIT['rad'][0])/8)
        quater = tf.transformations.quaternion_from_euler(ei, ej, ek)
        orient = Quaternion(quater[0], quater[1], quater[2], quater[3])

        object_pose = Pose(Point(x=pt_x, y=pt_y, z=0.5), orient)

        # spawn
        object_name = model_tag+"_"+str(i)
        spawn_model(object_name, object_xml, "", object_pose, "world")
        rospy.sleep(0.5)


if __name__ == '__main__':
    delete_model, spawn_model, object_monitor = setup_delete_spawn_service()
    print(get_object_list(object_monitor))
    # spawn_from_uniform(10, spawn_model)
    clean_floor(delete_model, object_monitor)
    spawn_from_gaussian(20, spawn_model)

