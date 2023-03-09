#!/usr/bin/env python

import rospy
import rospkg
import tf
import cv2
import numpy as np
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
import os
import datetime
import math as m
import pickle
import glob
import time

DEG2RAD = m.pi/180

yolo_data_generator=None

class Ranges:
    def __init__(self, dist_range, elevation_range, azimuth_range, yaw_range, pitch_range, roll_range):
        self.dist_range = dist_range
        self.elevation_range = elevation_range
        self.azimuth_range = azimuth_range
        self.yaw_range = yaw_range
        self.pitch_range = pitch_range
        self.roll_range = roll_range 

    def __len__(self):
        dist_count = len(self.dist_range)
        elevation_count = len(self.elevation_range)
        azimuth_count = len(self.azimuth_range)
        yaw_count = len(self.yaw_range)
        pitch_count = len(self.pitch_range)
        roll_count = len(self.roll_range)
        return dist_count * elevation_count * azimuth_count * yaw_count * pitch_count * roll_count

    def get_xyz_ypr(self, image_index):
        dist_count = len(self.dist_range)
        elevation_count = len(self.elevation_range)
        azimuth_count = len(self.azimuth_range)
        yaw_count = len(self.yaw_range)
        pitch_count = len(self.pitch_range)
        roll_count = len(self.roll_range)
        i = image_index

        i, remainder = divmod(i, dist_count)
        dist = self.dist_range[remainder]

        i, remainder = divmod(i, elevation_count)
        elevation = self.elevation_range[remainder] + np.random.uniform(0, 2*DEG2RAD)

        i, remainder = divmod(i, azimuth_count)
        azimuth = self.azimuth_range[remainder] + np.random.uniform(-4*DEG2RAD, 4*DEG2RAD)

        i, remainder = divmod(i, yaw_count)
        yaw = self.yaw_range[remainder]

        i, remainder = divmod(i, pitch_count)
        pitch = self.pitch_range[remainder]

        i, remainder = divmod(i, roll_count)
        roll = self.roll_range[remainder]

        z = dist*m.sin(elevation)
        proj_len = dist*m.cos(elevation)
        x = proj_len*m.cos(azimuth)
        y = proj_len*m.sin(azimuth)

        # return dist, elevation, azimuth, yaw, pitch, roll
        return x,y,z+0.1, azimuth-m.pi, pitch+elevation, roll
        

class YoloDataGenerator:
    def __init__(self, camera_name, target_name, target_bb_corners_3d, ranges: Ranges, datasets_directory= "./datasets", dataset_name="dataset", target_size=1.0, wait_time=0.1):
        self.ranges = ranges
        self.target_bb_corners_3d = target_bb_corners_3d
        self.camera_name = camera_name
        self.target_name = target_name
        self.bridge = CvBridge()
        self.image_idx = 0
        self.wait_time = wait_time
        self.set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_model_state_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.camera_info_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.dataset_dir_path = os.path.join(datasets_directory, dataset_name)
        self.metadata_path = os.path.join(self.dataset_dir_path, "metadata.pickle")
        try:
            os.makedirs(self.dataset_dir_path, exist_ok=False)
            self.save_metadata()

        except OSError:
            if not os.path.exists(self.metadata_path):
                raise Exception("Could not find metadata.pickle, broken dataset. Restart with new dataset")
            else:
                print("Dataset found, loading")
                self.load_metadata()
        self.length = len(self.ranges)
        print(len(ranges))

    def camera_info_callback(self, camera_info_msg):
        self.camera_info = camera_info_msg
        self.camera_matrix = np.reshape(camera_info_msg.K, (3, 3))


    def loop(self):
        if self.image_idx >= self.length:
            exit()
        x,y,z, yaw, pitch, roll = ranges.get_xyz_ypr(self.image_idx)

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        model_state = ModelState()
        model_state.model_name = self.camera_name
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = z
        model_state.pose.orientation.x = quat[0]
        model_state.pose.orientation.y = quat[1]
        model_state.pose.orientation.z = quat[2]
        model_state.pose.orientation.w = quat[3]
        
        try:
            model_state
            self.set_model_state_client(model_state)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return
        
        # wait for the model state to settle
        rospy.sleep(self.wait_time)
        
        # try:
        #     (trans, rot) = self.listener.lookupTransform('/camera_link', '/target_object', rospy.Time(0))
        # except:
        #     continue
        # transform the target corners to the camera frame
        camera_rot = None
        camera_trans = None
        image = None
        try:
            resp = self.get_model_state_client(self.target_name, self.camera_name)
            image =  np.copy(self.image)
            quat = resp.pose.orientation
            pos = resp.pose.position
            camera_rot = [quat.x, quat.y, quat.z, quat.w]
            camera_trans = [pos.x, pos.y, pos.z]
            # print(resp)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            return
        camera_rot_mtx = tf.transformations.quaternion_matrix(camera_rot)[0:3, 0:3]
        # trans = np.array([x,y,z])
        target_bb_corners_3d_camera = np.dot(camera_rot_mtx, self.target_bb_corners_3d.T).T + np.array(camera_trans)
        frame_transform_mtx = np.array([[0,-1, 0],
                                        [0, 0,-1],
                                        [1, 0, 0]])
        target_bb_corners_3d_camera = (frame_transform_mtx @ target_bb_corners_3d_camera.T).T

        # project the corners onto the image plane
        target_bb_corners_2d = np.dot(self.camera_matrix, target_bb_corners_3d_camera.T).T
        target_bb_corners_2d[:, 0] /= target_bb_corners_2d[:, 2]
        target_bb_corners_2d[:, 1] /= target_bb_corners_2d[:, 2]
        cv_image = np.copy(image)
        for center in target_bb_corners_2d[:, 0:2]:
            cv_image = cv2.circle(cv_image, center.astype(int), 3, (0,255,0), thickness=-1)

        # calculate the bounding box for the target object
        min_x = int(np.min(target_bb_corners_2d[:, 0]))
        min_y = int(np.min(target_bb_corners_2d[:, 1]))
        max_x = int(np.max(target_bb_corners_2d[:, 0]))
        max_y = int(np.max(target_bb_corners_2d[:, 1]))
        
        # draw the bounding box on the image
        cv_image = cv2.rectangle(cv_image, (min_x, min_y), (max_x, max_y), (0, 0, 255), 2)
        cv2.imshow("winname", cv_image)
        cv2.waitKey(1)
        
        # save the image and bounding box
        image_filename = 'image_{:06d}.jpg'.format(self.image_idx)
        cv2.imwrite(os.path.join(self.dataset_dir_path, image_filename), image)
        
        label_filename = 'image_{:06d}.txt'.format(self.image_idx)
        with open(os.path.join(self.dataset_dir_path, label_filename), 'w') as f:
            f.write(f"{min_x} {min_y} {max_x} {max_y}\n")
            f.write(f"{camera_trans}\n")
            f.write(f"{camera_rot}\n")
        secs = (self.length-self.image_idx)*self.wait_time
        mins, secs = divmod(secs, 60)

        progress = self.image_idx/self.length
        print(f"{self.image_idx}/{self.length} : {progress}% ETA: {int(mins)}:{int(secs)}")
        self.image_idx += 1
            
    def image_callback(self, image_msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
    
    def save_metadata(self):
        with open(self.metadata_path, "wb") as file:
            obj = (self.image_idx, self.ranges, self.target_bb_corners_3d)
            print(obj)
            pickle.dump(obj, file)
    
    def load_metadata(self):
        with open(self.metadata_path, "rb") as file:
            self.image_idx, self.ranges, self.target_bb_corners_3d = pickle.load(file)
            print(self.image_idx)
            print(self.ranges)

def on_shutdown():
    print("Shutting down")
    yolo_data_generator.save_metadata() 

def my_lin(lb, ub, steps, spacing=1.4):
    span = (ub-lb)
    dx = 1.0 / (steps-1)
    return [lb + (i*dx)**spacing*span for i in range(steps)]

if __name__ == '__main__':
    rospy.init_node('yolo_data_generator')
    target_bb_corners = np.array([
        [-0.5, -0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [-0.5, 0.5, -0.5],
        [-0.5, 0.5, 0.5],
        [0.5, -0.5, -0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, -0.5],
        [0.5, 0.5, 0.5]
        ])
    target_bb_corners *= (np.array([[2.6714, 0.3910, 0.5807]])*0.9)
    target_bb_corners += np.array([[0,0,0.2]])

    ranges = Ranges(dist_range=my_lin(2, 100, 20, spacing=1.4), elevation_range=np.linspace(0*DEG2RAD, 25*DEG2RAD, 10), azimuth_range=np.linspace(0, 360*DEG2RAD, 20), yaw_range=np.linspace(0, 0, 1), pitch_range=np.linspace(0, 0, 1), roll_range=np.linspace(0, 0, 1))
    yolo_data_generator = YoloDataGenerator("camera", "target_object", target_bb_corners, ranges, wait_time=5/60)
    rospy.on_shutdown(on_shutdown)
    while not rospy.is_shutdown():
        yolo_data_generator.loop()
    rospy.spin()
