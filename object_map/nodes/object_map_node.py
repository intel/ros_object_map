#!/usr/bin/env python
"""
 Copyright (c) 2017 Intel Corporation

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
"""
import numpy as np
import rospy
import tf
from std_msgs.msg import Int32
from geometry_msgs.msg import Point

from object_map.msg import Objects
from object_map.msg import ObjectInfo

from object_map.object_filter import ObjectFilter
from object_map.object_marker import ObjectMarker
from object_map.conf import Param


class ObjectMap(object):
    """
    ObjectMap Class
    """
    def __init__(self):
        rospy.init_node('object_map')
        rospy.loginfo('Start object_map')

        self._tf_tree = tf.TransformListener()

        # Load configuration
        self._param = Param()
        self._min_diameter = self._param.min_diameter
        self._max_diameter = self._param.max_diameter
        self._max_distance = self._param.max_distance
        self._ref_camera_id = self._param.ref_camera_id
        self._ref_map_id = self._param.ref_map_id
        self._default_distance_tolerance = self._param.default_distance_tolerance
        self._default_probability_tolerance = self._param.default_probability_tolerance
        self._object_marker_file = self._param.object_map_file
        self._object_type_list = self._param.object_type_list

        rospy.loginfo('  Param: min_diameter = ' + str(self._min_diameter))
        rospy.loginfo('  Param: max_diameter = ' + str(self._max_diameter))
        rospy.loginfo('  Param: max_distance = ' + str(self._max_distance))
        rospy.loginfo('  Param: ref_camera_id = ' + str(self._ref_camera_id))
        rospy.loginfo('  Param: ref_map_id = ' + str(self._ref_map_id))
        rospy.loginfo('  Param: default_distance_tolerance = ' + str(self._default_distance_tolerance))
        rospy.loginfo('  Param: default_probability_tolerance = ' + str(self._default_probability_tolerance))
        rospy.loginfo('  Param: object_map_file = ' + str(self._object_marker_file))
        rospy.loginfo('  Param: object_type_list = ' + str(self._object_type_list))

        if self._min_diameter is None and self._max_diameter is None and self._object_marker_file is None:
            rospy.logerr('No Param found, fail to load config.xml')
            return

        # Subscribe object topics and publish filtered object topic
        ObjectFilter()

        # Init object map
        self._object_marker = ObjectMarker()

        # Load exist map
        self._object_infos = self._object_marker.load_map(self._object_marker_file)

        # rospy.loginfo('object_infos:\n' + str(self._object_infos))
        if self._object_infos is not None:
            rospy.loginfo('Total markers: ' + str(len(self._object_infos)))
            # waiting for 2s for rviz Marker connection.
            rospy.sleep(2)
            self._object_marker.display_in_rviz(self._ref_map_id, self._object_infos)

        # Subscribe filtered objects
        rospy.loginfo('Waiting NEW objects...')
        rospy.Subscriber('/object_map/filtered_object', Objects, self._object_callback, queue_size=1)
        # Subscibe map saving
        rospy.Subscriber('/object_map/map_save', Int32, self._save_callback, queue_size=1)

    def _object_callback(self, object_list):
        """
        Collect available objects to object_infos
        """
        shot_ts = object_list.header.stamp

        for obj in object_list.Objects:
            rospy.loginfo('Receive object [type]:' + str(obj.name) + ', [id]:' + str(obj.id))
            if not self._check_object_type(obj.name):
                continue

            if not self._check_confidence(obj.name, obj.probability):
                continue

            if not self._check_max_distance(obj):
                continue

            # Collect new object
            if self._check_new_object(obj, shot_ts):
                rospy.loginfo('+++ New object [type]: ' + str(obj.name) + ', [id]: ' + str(obj.id))
                object_info = self._object_collector(obj, shot_ts)
                self._object_infos.append(object_info)
                # rospy.loginfo('Target objects: ' + str(self._object_infos))
                self._object_marker.display_in_rviz(self._ref_map_id, self._object_infos)

            rospy.loginfo('Total markers: ' + str(len(self._object_infos)))

    def _save_callback(self, msg):
        """
        Execute save map when subscribed "save" topic
        """
        if msg.data == 1:
            self._object_marker.save_map(self._object_marker_file, self._object_infos)

    def _check_object_type(self, object_type):
        """
        check if the object type is or not in support type lists
        """
        return object_type in self._object_type_list

    def _check_confidence(self, name, probability):
        """
        check if probability is or not over default value
        """
        probability_tolerance = self._param.get_object_probability_tolerance(name)
        if probability > probability_tolerance:
            return True
        rospy.loginfo('Ignore [small probability]: ' + str(probability))
        return False

    def _check_max_distance(self, obj):
        """
        check if object is in available distance
        """
        object_x = (obj.min_point.x + obj.max_point.x) / 2
        object_y = (obj.min_point.y + obj.max_point.y) / 2
        object_z = (obj.min_point.z + obj.max_point.z) / 2
        distance = np.sqrt(object_x ** 2 + object_y ** 2 + object_z ** 2)

        if distance < self._max_distance:
            return True
        rospy.loginfo('Ignore [big distance]: ' + str(distance))
        return False

    def _check_new_object(self, obj, shot_ts):
        """
        check if the object is new or old one
        add new one and ignore old one
        """
        # Ignore same ID
        for target in self._object_infos:
            if obj.id == target.id:
                rospy.loginfo('Ignore [same ID]: ' + str(obj.id))
                return False

        # Ignore same distance of one type
        for target in self._object_infos:
            if obj.name != target.type:
                continue

            rospy.loginfo('target id = ' + str(target.id))
            rospy.loginfo('new id = ' + str(obj.id))

            # translate obj coordinate from camera to map
            pmin = self._object_to_map(obj.min_point, shot_ts, self._ref_camera_id, self._ref_map_id)
            pmax = self._object_to_map(obj.max_point, shot_ts, self._ref_camera_id, self._ref_map_id)

            tmp_x = (pmin.x + pmax.x) / 2
            tmp_y = (pmin.y + pmax.y) / 2

            # calculate the target coordinat center in the map
            center_x = (target.points[0].x + target.points[1].x) / 2
            center_y = (target.points[0].y + target.points[1].y) / 2

            map_x = abs(tmp_x - center_x)
            map_y = abs(tmp_y - center_y)

            distance_tolerance = self._param.get_object_distance_tolerance(obj.name)
            if map_x < distance_tolerance and map_y < distance_tolerance:
                # Recognize as same object when id changed but type and location not changed, update the id
                target.id = obj.id
                rospy.loginfo('Ignore [same distance]: ' + str(map_x) + ', ' + str(map_y))
                return False
        return True

    def _object_to_map(self, point3d, shot_ts, ref_camera_id, ref_map_id):
        """
        switch camera points to map points
        """

        object_x = point3d.x
        object_y = point3d.y
        object_z = point3d.z

        # use rgbd_cam as ref coordinate
        # it's not same as realsense_cam, but it works well after validation
        (trans, rot) = self._tf_tree.lookupTransform(ref_map_id, ref_camera_id, shot_ts)

        translation = np.matrix(tf.TransformerROS().fromTranslationRotation(trans, rot))

        object_pose = np.matrix(tf.TransformerROS().fromTranslationRotation((object_x, object_y, object_z),
                                                                            (0, 0, 0, 1)))
        map_pose = translation * object_pose

        (map_x, map_y, map_z) = tf.transformations.translation_from_matrix(map_pose)
        return Point(map_x, map_y, map_z)

    def _object_collector(self, obj, shot_ts):
        """
        collector new object in object_infos
        """
        object_info = ObjectInfo()
        object_info.id = obj.id
        object_info.type = obj.name
        point1 = self._object_to_map(obj.min_point, shot_ts, self._ref_camera_id, self._ref_map_id)
        point2 = self._object_to_map(obj.max_point, shot_ts, self._ref_camera_id, self._ref_map_id)
        object_info.points.append(point1)
        object_info.points.append(point2)
        len_x = abs(point1.x - point2.x)
        len_y = abs(point1.y - point2.y)
        diameter = max(self._min_diameter, min(self._max_diameter, max(len_y, len_x)))
        object_info.scale.x = diameter
        object_info.scale.y = diameter
        object_info.scale.z = 0.1

        center_x = (point1.x + point2.x) / 2
        center_y = (point1.y + point2.y) / 2

        object_info.pose.position.x = center_x
        object_info.pose.position.y = center_y
        object_info.pose.position.z = 0
        object_info.pose.orientation.x = 0
        object_info.pose.orientation.y = 0
        object_info.pose.orientation.z = 0
        object_info.pose.orientation.w = 1

        return object_info

if __name__ == '__main__':
    try:
        ObjectMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_map node terminated.")
