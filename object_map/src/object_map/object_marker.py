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
import os
import copy
import rospy
import yaml
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from object_map.msg import ObjectInfo


class ObjectMarker(object):
    """
    ObjectMarker class
    """
    def __init__(self):
        rospy.loginfo('Init ObjectMarker()')

        # Initial default marker properties
        self.marker = Marker()
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.scale.x = 0
        self.marker.scale.y = 0
        self.marker.scale.z = 0

        line_color = ColorRGBA()
        line_color.r = 1.0
        line_color.g = 1.0
        line_color.b = 0.0
        line_color.a = 1.0
        self.marker.colors.append(line_color)
        self.marker.colors.append(line_color)

        # Marker topic on Rviz
        self._markers_pub = rospy.Publisher('/object_map/Markers', MarkerArray, queue_size=1)

    def save_map(self, map_file, object_infos):
        """
        save object_infos to map file
        """
        rospy.loginfo('Save map to ' + str(map_file))
        if map_file is None:
            rospy.logwarn('save_map: map is null, stop to save map')
            return

        map_folder = os.path.split(map_file)[0]
        if not os.access(map_folder, os.W_OK):
            rospy.logwarn('save_map: has no permission save to ' + str(map_file))
            return

        rospy.loginfo('Save map to ' + str(map_file))
        rospy.loginfo('object_infos: ' + str(object_infos))
        with open(map_file, 'w') as tmp_file:
            marker_dict_list = []
            for marker in object_infos:
                marker_dict = {'id': marker.id, 'type': marker.type, 'scale':
                               [marker.scale.x, marker.scale.y, marker.scale.z],
                               'pose': [float(marker.pose.orientation.w),
                                        float(marker.pose.position.x),
                                        float(marker.pose.position.y),
                                        float(marker.pose.position.z)],
                               'points': [float(marker.points[0].x),
                                          float(marker.points[0].y),
                                          float(marker.points[0].z),
                                          float(marker.points[1].x),
                                          float(marker.points[1].y),
                                          float(marker.points[1].z)]}
                marker_dict_list.append(marker_dict)
            tmp_file.write(yaml.dump_all(marker_dict_list))

    def load_map(self, map_file):
        """
        Load object maps from a exist yaml file
        No detection or exception handling for FileNotExist exception
        """
        object_infos = []
        if map_file is None:
            rospy.loginfo('load_map: map is null, return None')
            return object_infos

        if not os.path.exists(map_file):
            rospy.loginfo('load_map: Map not exist: ' + str(map_file))
            return object_infos

        rospy.loginfo('Load map: ' + str(map_file))
        with open(map_file, 'r') as tmp_file:
            for i in yaml.load_all(tmp_file):
                try:
                    object_info = ObjectInfo()
                    object_info.id = i['id']
                    object_info.type = i['type']
                    object_info.scale.x = i['scale'][0]
                    object_info.scale.y = i['scale'][1]
                    object_info.scale.z = i['scale'][2]
                    object_info.pose.orientation.w = i['pose'][0]
                    object_info.pose.position.x = i['pose'][1]
                    object_info.pose.position.y = i['pose'][2]
                    object_info.pose.position.z = i['pose'][3]

                    point1 = Point()
                    point1.x = i['points'][0]
                    point1.y = i['points'][1]
                    point1.z = i['points'][2]
                    object_info.points.append(point1)

                    point2 = Point()
                    point2.x = i['points'][3]
                    point2.y = i['points'][4]
                    point2.z = i['points'][5]
                    object_info.points.append(point2)
                    object_infos.append(object_info)
                except KeyError:
                    rospy.loginfo('load_maps KeyError')
        return object_infos

    def display_in_rviz(self, frame_id, object_infos):
        """
        publish objects to marker topic and display in rviz
        """
        pub_markers = MarkerArray()
        for object_info in object_infos:
            marker = copy.deepcopy(self.marker)
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = frame_id
            marker.action = Marker.ADD
            marker.type = Marker.CYLINDER
            marker.ns = "SematicSLAM_line"
            marker.scale.z = 0.01
            marker.color.a = 0.35

            marker.id = object_info.id
            marker.text = object_info.type
            marker.points = object_info.points
            marker.scale.x = object_info.scale.x
            marker.scale.y = object_info.scale.x
            marker.scale.z = object_info.scale.y
            marker.pose.position.x = object_info.pose.position.x
            marker.pose.position.y = object_info.pose.position.y
            marker.pose.position.z = object_info.pose.position.z
            marker.pose.orientation.x = object_info.pose.orientation.x
            marker.pose.orientation.y = object_info.pose.orientation.y
            marker.pose.orientation.z = object_info.pose.orientation.z
            marker.pose.orientation.w = object_info.pose.orientation.w

            pub_markers.markers.append(marker)

        for object_info in object_infos:
            marker = copy.deepcopy(self.marker)
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = frame_id
            marker.action = Marker.ADD
            marker.type = Marker.TEXT_VIEW_FACING
            marker.ns = "SematicSLAM_text"
            marker.scale.z = 0.3
            marker.color.a = 1.0

            marker.id = object_info.id
            marker.text = object_info.type
            marker.points = object_info.points
            marker.scale.x = object_info.scale.x
            marker.scale.y = object_info.scale.x
            marker.scale.z = object_info.scale.y
            marker.pose.position.x = object_info.pose.position.x
            marker.pose.position.y = object_info.pose.position.y
            marker.pose.position.z = object_info.pose.position.z
            marker.pose.orientation.x = object_info.pose.orientation.x
            marker.pose.orientation.y = object_info.pose.orientation.y
            marker.pose.orientation.z = object_info.pose.orientation.z
            marker.pose.orientation.w = object_info.pose.orientation.w

            pub_markers.markers.append(marker)

        # Publish markers on Rviz
        rospy.loginfo('Publish markers to Topic: /object_map/Markers')
        self._markers_pub.publish(pub_markers)
