#!/usr/bin/env python3

import rospy
import numpy as np
import random
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker

frequency = 1
transform_camera_origin = None
transform_buffer = None
transform_listener = None
transform_origin = 'map'
transform_camera = 'base_scan'
found_tags = {}
file_ = ""

def get_transform(A, B):
    global tf_buffer
    try:
        pose = tf_buffer.lookup_transform(A, B, rospy.Time(0), rospy.Duration(4))
    except Exception as e:
        print('[-]', e)
        return
    
    transform_a = [pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z]
    transform_b = (
        pose.transform.rotation.x,
        pose.transform.rotation.y,
        pose.transform.rotation.z,
        pose.transform.rotation.w)
    r = Rotation.from_quat(transform_b).as_matrix()

    # Make affine matrix for transformation
    return np.array([[r[0][0], r[0][1], r[0][2], transform_a[0]],
                     [r[1][0], r[1][1], r[1][2], transform_a[1]],
                     [r[2][0], r[2][1], r[2][2], transform_a[2]],
                     [0, 0, 0, 1]])


def get_tag_detection(tag_msg):
    """
    Detect an AprilTag's pose relative to the camera and update the list if something was detected.
    """
    # Verify there is at least one tag detected
    if not tag_msg.detections:
        return

    for detection in tag_msg.detections:
        tag_id = detection.id
        tag_pose = detection.pose.pose.pose
        # Use this to make goal pose in robot base frame
        t = [tag_pose.position.x, tag_pose.position.y, tag_pose.position.z]
        q = [tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z]
        # Make it into an affine matrix
        r = Rotation.from_quat(q).as_matrix()


        T_AC = np.array([[r[0][0], r[0][1], r[0][2], t[0]],
                         [r[1][0], r[1][1], r[1][2], t[1]],
                         [r[2][0], r[2][1], r[2][2], t[2]],
                         [0, 0, 0, 1]])

        if transform_camera_origin is None:
            print("[-] Unable to create global transform")
            return
        T_AO = T_AC @ transform_camera_origin
        if tag_id in found_tags:
            print('[+] Updating the tag position : ', tag_id)
            L = 0.9
            found_tags[tag_id] = np.add(found_tags[tag_id], T_AO)
        else:
            print(f'[+] New tag with tag id {tag_id} found.')
            found_tags[tag_id] = T_AO


def callback(event):
    global transform_camera_origin
    transform_camera_origin = get_transform(transform_camera, transform_origin)
    if found_tags != {}:
        tags = []
        for id in found_tags.keys():
            print("[+]", id, found_tags[id])
            tags.append("id: " + str(id))
            for row in found_tags[id]:
                tags.append('---+---+---+---+---')
        np.savetxt(file_, tags, fmt="%s", delimiter=",")

    


def tracking_node():
    global transform_listener, transform_buffer, file_
    rospy.init_node('tag_tracking_node')
    robot_name = f'splinter_{random.randint(1000,10000) * random.randint(50,500)}'
    print(f"[+] Saving april tag sequence in : {robot_name}.txt")
    file_ = f'{robot_name}.txt'
    transform_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1))
    transform_listener = tf2_ros.TransformListener(transform_buffer)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, get_tag_detection, queue_size=10)
    rospy.Timer(rospy.Duration(frequency), callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        tracking_node()
    except rospy.ROSInterruptException:
        pass