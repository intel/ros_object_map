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
import rospy
from geometry_msgs.msg import Point
from object_map.msg import ObjectInfo


class Param(object):
    """
    parameters
    """
    def __init__(self):
        self._min_diameter = None
        self._max_diameter = None
        self._max_distance = None
        self._ref_camera_id = None
        self._ref_map_id = None
        self._default_distance_tolerance = None
        self._default_probability_tolerance = None
        self._object_map_file = None
        self._object_type_dict = {}
        self._object_type_list = []

        self._config_from_param()

    def _config_from_param(self):
        """
        Read configure from rosparam
        """
        rospy.loginfo("Init Param()")
        if rospy.has_param("~min_diameter"):
            self._min_diameter = rospy.get_param("~min_diameter")

        if rospy.has_param("~max_diameter"):
            self._max_diameter = rospy.get_param("~max_diameter")

        if rospy.has_param("~max_distance"):
            self._max_distance = rospy.get_param("~max_distance")

        if rospy.has_param("~ref_camera_id"):
            self._ref_camera_id = rospy.get_param("~ref_camera_id")

        if rospy.has_param("~ref_map_id"):
            self._ref_map_id = rospy.get_param("~ref_map_id")

        if rospy.has_param("~default_distance_tolerance"):
            self._default_distance_tolerance = rospy.get_param("~default_distance_tolerance")

        if rospy.has_param("~default_probability_tolerance"):
            self._default_probability_tolerance = rospy.get_param("~default_probability_tolerance")

        if rospy.has_param("~object_map_file"):
            self._object_map_file = rospy.get_param("~object_map_file")

        if rospy.has_param("~object_type_list"):
            self._object_type_list = rospy.get_param("~object_type_list")

        if rospy.has_param("~objects"):
            self._object_type_dict = rospy.get_param("~objects")
            self._object_type_list = self._object_type_dict.keys()

    @property
    def min_diameter(self):
        """
        get min diameter
        """
        return self._min_diameter

    @property
    def max_diameter(self):
        """
        get max diameter
        """
        return self._max_diameter

    @property
    def max_distance(self):
        """
        get max distance
        """
        return self._max_distance

    @property
    def ref_camera_id(self):
        """
        get reference camera id
        """
        return self._ref_camera_id

    @property
    def ref_map_id(self):
        """
        get reference map id
        """
        return self._ref_map_id

    @property
    def default_distance_tolerance(self):
        """
        get default distance tolerance
        """
        return self._default_distance_tolerance

    @property
    def default_probability_tolerance(self):
        """
        get default probability tolerance
        """
        return self._default_probability_tolerance

    @property
    def object_map_file(self):
        """
        get object map file
        """
        return self._object_map_file

    @property
    def object_type_list(self):
        """
        get supported object types
        """
        return self._object_type_list

    def get_object_distance_tolerance(self, object_type):
        """
        get distance tolerance of selected object
        """
        if not self._object_type_dict:
            return self.default_distance_tolerance

        if 'distance_tolerance' in self._object_type_dict[object_type]:
            return self._object_type_dict[object_type]['distance_tolerance']
        return self.default_distance_tolerance

    def get_object_probability_tolerance(self, object_type):
        """
        get probability tolerance of selected object
        """
        if not self._object_type_dict:
            return self.default_probability_tolerance

        if 'probability' in self._object_type_dict[object_type]:
            return self._object_type_dict[object_type]["probability"]
        return self.default_probability_tolerance
