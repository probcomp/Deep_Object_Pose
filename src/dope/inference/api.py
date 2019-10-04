#!/usr/bin/env python

# Copyright (c) 2018 NVIDIA Corporation. All rights reserved.
# This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
# https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

"""
This file starts a ROS node to run DOPE, 
listening to an image topic and publishing poses.
"""

from __future__ import print_function

import cv2
import numpy as np
import transforms3d as tf3d
from PIL import Image
from PIL import ImageDraw
from cuboid import Cuboid3d
from cuboid_pnp_solver import CuboidPNPSolver
from detector import ModelData, ObjectDetector
from yaml import load


class Draw(object):
    """Drawing helper class to visualize the neural network output"""

    def __init__(self, im):
        """
        :param im: The image to draw in.
        """
        self.draw = ImageDraw.Draw(im)

    def draw_line(self, point1, point2, line_color, line_width=2):
        """Draws line on image"""
        if point1 is not None and point2 is not None:
            self.draw.line([point1, point2], fill=line_color, width=line_width)

    def draw_dot(self, point, point_color, point_radius):
        """Draws dot (filled circle) on image"""
        if point is not None:
            xy = [
                point[0] - point_radius,
                point[1] - point_radius,
                point[0] + point_radius,
                point[1] + point_radius
            ]
            self.draw.ellipse(xy,
                              fill=point_color,
                              outline=point_color
                              )

    def draw_cube(self, points, color=(255, 0, 0)):
        """
        Draws cube with a thick solid line across
        the front top edge and an X on the top face.
        """

        # draw front
        self.draw_line(points[0], points[1], color)
        self.draw_line(points[1], points[2], color)
        self.draw_line(points[3], points[2], color)
        self.draw_line(points[3], points[0], color)

        # draw back
        self.draw_line(points[4], points[5], color)
        self.draw_line(points[6], points[5], color)
        self.draw_line(points[6], points[7], color)
        self.draw_line(points[4], points[7], color)

        # draw sides
        self.draw_line(points[0], points[4], color)
        self.draw_line(points[7], points[3], color)
        self.draw_line(points[5], points[1], color)
        self.draw_line(points[2], points[6], color)

        # draw dots
        self.draw_dot(points[0], point_color=color, point_radius=4)
        self.draw_dot(points[1], point_color=color, point_radius=4)

        # draw x on the top
        self.draw_line(points[0], points[5], color)
        self.draw_line(points[1], points[4], color)


class Dope(object):
    """ROS node that listens to image topic, runs DOPE, and publishes DOPE results"""
    def __init__(self, config_f):
        with open(config_f, 'r') as f:
            self.params = load(f)

        # self.pubs = {}
        self.models = {}
        self.pnp_solvers = {}
        # self.pub_dimension = {}
        self.draw_colors = {}
        self.dimensions = {}
        self.class_ids = {}
        self.model_transforms = {}
        self.meshes = {}
        self.mesh_scales = {}

        self.input_is_rectified = True
        self.downscale_height = 500

        self.config_detect = lambda: None
        self.config_detect.mask_edges = 1
        self.config_detect.mask_faces = 1
        self.config_detect.vertex = 1
        self.config_detect.threshold = 0.5
        self.config_detect.softmax = 1000
        self.config_detect.thresh_angle = 0.5
        self.config_detect.thresh_map = 0.01
        self.config_detect.sigma = 3
        self.config_detect.thresh_points = 0.1

        # For each object to detect, load network model, create PNP solver, and start ROS publishers
        for model, weights_filename in self.params['weights'].items():
            self.models[model] = \
                ModelData(
                    model,
                    weights_filename
                )
            self.models[model].load_net_model()

            try:
                M = np.array(self.params['model_transforms'][model], dtype='float64')
                self.model_transforms[model] = tf3d.quaternions.mat2quat(M)
            except KeyError:
                self.model_transforms[model] = np.array([0.0, 0.0, 0.0, 1.0], dtype='float64')

            try:
                self.meshes[model] = self.params['meshes'][model]
            except KeyError:
                pass

            try:
                self.mesh_scales[model] = self.params['mesh_scales'][model]
            except KeyError:
                self.mesh_scales[model] = 1.0

            self.draw_colors[model] = tuple(self.params["draw_colors"][model])
            self.dimensions[model] = tuple(self.params["dimensions"][model])
            self.class_ids[model] = self.params["class_ids"][model]

            self.pnp_solvers[model] = \
                CuboidPNPSolver(
                    model,
                    cuboid3d=Cuboid3d(self.params['dimensions'][model])
                )
            # self.pubs[model] = \
            #     rospy.Publisher(
            #         '{}/pose_{}'.format(rospy.get_param('~topic_publishing'), model),
            #         PoseStamped,
            #         queue_size=10
            #     )
            # self.pub_dimension[model] = \
            #     rospy.Publisher(
            #         '{}/dimension_{}'.format(rospy.get_param('~topic_publishing'), model),
            #         String,
            #         queue_size=10
            #     )

        # Start ROS publishers
        # self.pub_rgb_dope_points = \
        #     rospy.Publisher(
        #         rospy.get_param('~topic_publishing') + "/rgb_points",
        #         ImageSensor_msg,
        #         queue_size=10
        #     )
        # self.pub_detections = \
        #     rospy.Publisher(
        #         '~detected_objects',
        #         Detection3DArray,
        #         queue_size=10
        #     )
        # self.pub_markers = \
        #     rospy.Publisher(
        #         '~markers',
        #         MarkerArray,
        #         queue_size=10
        #     )

        # Start ROS subscriber
        # image_sub = message_filters.Subscriber(
        #     rospy.get_param('~topic_camera'),
        #     ImageSensor_msg
        # )
        # info_sub = message_filters.Subscriber(
        #     rospy.get_param('~topic_camera_info'),
        #     CameraInfo
        # )
        # ts = message_filters.TimeSynchronizer([image_sub, info_sub], 1)
        # ts.registerCallback(self.image_callback)

        # print("Running DOPE...  (Listening to camera topic: '{}')".format(rospy.get_param('~topic_camera')))
        # print("Ctrl-C to stop")

    def image_callback(self, img, camera_info):
        """Image detector"""

        # img = self.cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")
        # cv2.imwrite('img.png', cv2.cvtColor(img, cv2.COLOR_BGR2RGB))  # for debugging

        # Update camera matrix and distortion coefficients
        if self.input_is_rectified:
            P = np.matrix(camera_info.P, dtype='float64')
            P.resize((3, 4))
            camera_matrix = P[:, :3]
            dist_coeffs = np.zeros((4, 1))
        else:
            camera_matrix = np.matrix(camera_info.K, dtype='float64')
            camera_matrix.resize((3, 3))
            dist_coeffs = np.matrix(camera_info.D, dtype='float64')
            dist_coeffs.resize((len(camera_info.D), 1))

        # Downscale image if necessary
        height, width, _ = img.shape
        scaling_factor = float(self.downscale_height) / height
        if scaling_factor < 1.0:
            camera_matrix[:2] *= scaling_factor
            img = cv2.resize(img, (int(scaling_factor * width), int(scaling_factor * height)))

        for m in self.models:
            self.pnp_solvers[m].set_camera_intrinsic_matrix(camera_matrix)
            self.pnp_solvers[m].set_dist_coeffs(dist_coeffs)

        # Copy and draw image
        img_copy = img.copy()
        im = Image.fromarray(img_copy)
        draw = Draw(im)

        # detection_array = Detection3DArray()
        # detection_array.header = image_msg.header
        detection_array = []

        for m in self.models:
            # Detect object
            results = ObjectDetector.detect_object_in_image(
                self.models[m].net,
                self.pnp_solvers[m],
                img,
                self.config_detect
            )

            # Publish pose and overlay cube on image
            for i_r, result in enumerate(results):
                if result["location"] is None:
                    continue
                loc = result["location"]
                ori = result["quaternion"]

                # transform orientation
                transformed_ori = tf3d.quaternions.qmult(ori, self.model_transforms[m])

                # rotate bbox dimensions if necessary
                # (this only works properly if model_transform is in 90 degree angles)
                dims = rotate_vector(vector=self.dimensions[m], quaternion=self.model_transforms[m])
                dims = np.absolute(dims)
                dims = tuple(dims)

                # TODO: Convert pose_msg to detection

                x = loc[0] / CONVERT_SCALE_CM_TO_METERS
                y = loc[1] / CONVERT_SCALE_CM_TO_METERS
                z = loc[2] / CONVERT_SCALE_CM_TO_METERS
                qw = transformed_ori[3]
                qx = transformed_ori[0]
                qy = transformed_ori[1]
                qz = transformed_ori[2]
                detection = np.array([x, y, z, qw, qx, qy, qz])
                detection_array.append((m, detection))

                # pose_msg = PoseStamped()
                # pose_msg.header = image_msg.header
                # CONVERT_SCALE_CM_TO_METERS = 100
                # pose_msg.pose.position.x = loc[0] / CONVERT_SCALE_CM_TO_METERS
                # pose_msg.pose.position.y = loc[1] / CONVERT_SCALE_CM_TO_METERS
                # pose_msg.pose.position.z = loc[2] / CONVERT_SCALE_CM_TO_METERS
                # pose_msg.pose.orientation.x = transformed_ori[0]
                # pose_msg.pose.orientation.y = transformed_ori[1]
                # pose_msg.pose.orientation.z = transformed_ori[2]
                # pose_msg.pose.orientation.w = transformed_ori[3]

                # Publish
                # self.pubs[m].publish(pose_msg)
                # self.pub_dimension[m].publish(str(dims))

                # Add to Detection3DArray
                # detection = Detection3D()
                # hypothesis = ObjectHypothesisWithPose()
                # hypothesis.id = self.class_ids[result["name"]]
                # hypothesis.score = result["score"]
                # hypothesis.pose.pose = pose_msg.pose
                # detection.results.append(hypothesis)
                # detection.bbox.center = pose_msg.pose
                # detection.bbox.size.x = dims[0] / CONVERT_SCALE_CM_TO_METERS
                # detection.bbox.size.y = dims[1] / CONVERT_SCALE_CM_TO_METERS
                # detection.bbox.size.z = dims[2] / CONVERT_SCALE_CM_TO_METERS
                # detection_array.detections.append(detection)

                # Draw the cube
                if None not in result['projected_points']:
                    points2d = []
                    for pair in result['projected_points']:
                        points2d.append(tuple(pair))
                    draw.draw_cube(points2d, self.draw_colors[m])

        return detection_array

        # Publish the image with results overlaid
        # self.pub_rgb_dope_points.publish(
        #     CvBridge().cv2_to_imgmsg(
        #         np.array(im)[..., ::-1],
        #         "bgr8"
        #     )
        # )
        # self.pub_detections.publish(detection_array)
        # self.publish_markers(detection_array)

    def publish_markers(self, detection_array):
        # Delete all existing markers
        markers = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.pub_markers.publish(markers)

        # Object markers
        class_id_to_name = {class_id: name for name, class_id in self.class_ids.iteritems()}
        markers = MarkerArray()
        for i, det in enumerate(detection_array.detections):
            name = class_id_to_name[det.results[0].id]
            color = self.draw_colors[name]

            # cube marker
            marker = Marker()
            marker.header = detection_array.header
            marker.action = Marker.ADD
            marker.pose = det.bbox.center
            marker.color.r = color[0] / 255.0
            marker.color.g = color[1] / 255.0
            marker.color.b = color[2] / 255.0
            marker.color.a = 0.4
            marker.ns = "bboxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.scale = det.bbox.size
            markers.markers.append(marker)

            # text marker
            marker = Marker()
            marker.header = detection_array.header
            marker.action = Marker.ADD
            marker.pose = det.bbox.center
            marker.color.r = color[0] / 255.0
            marker.color.g = color[1] / 255.0
            marker.color.b = color[2] / 255.0
            marker.color.a = 1.0
            marker.id = i
            marker.ns = "texts"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.text = '{} ({:.2f})'.format(name, det.results[0].score)
            markers.markers.append(marker)

            # mesh marker
            try:
                marker = Marker()
                marker.header = detection_array.header
                marker.action = Marker.ADD
                marker.pose = det.bbox.center
                marker.color.r = color[0] / 255.0
                marker.color.g = color[1] / 255.0
                marker.color.b = color[2] / 255.0
                marker.color.a = 0.7
                marker.ns = "meshes"
                marker.id = i
                marker.type = Marker.MESH_RESOURCE
                marker.scale.x = self.mesh_scales[name]
                marker.scale.y = self.mesh_scales[name]
                marker.scale.z = self.mesh_scales[name]
                marker.mesh_resource = self.meshes[name]
                markers.markers.append(marker)
            except KeyError:
                # user didn't specify self.meshes[name], so don't publish marker
                pass

        self.pub_markers.publish(markers)


def rotate_vector(vector, quaternion):
    q_conj = tf3d.quaternions.qconjugate(quaternion)
    vector = np.array(vector, dtype='float64')
    vector = np.append(vector, [0.0])
    vector = tf3d.quaternions.qmult(q_conj, vector)
    vector = tf3d.quaternions.qmult(vector, quaternion)
    return vector[:3]


def main():
    """Main routine to run DOPE"""

    # Initialize ROS node
    # rospy.init_node('dope')

    Dope("../../../config/config_pose.yaml")

    # try:
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass


if __name__ == "__main__":
    main()
