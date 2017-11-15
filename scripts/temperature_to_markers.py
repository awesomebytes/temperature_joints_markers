#!/usr/bin/env python

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rospy
from get_mesh_urdf import get_link_mesh_info
from visualization_msgs.msg import Marker, MarkerArray

"""
Markers showing the mesh and a text over it
on every motor configured in the yaml file with
the color temperature.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class TemperatureMarkers(object):
    def __init__(self):
        self.config = rospy.get_param('temperature_joints', None)
        self.general_threshold = rospy.get_param('temperature_joints_general_publication_threshold', 0.0)
        if self.config is None:
            rospy.logerr("No configuration found in param server")

        self.marker_id = 1234
        self.marker_dict = {}
        for elem in self.config:
            rospy.loginfo("Configuring: " + str(elem))
            # - motor_name: gripper_left_motor
            #   link_to_show: gripper_left_base_link
            #   min_temperature: 20.0
            #   max_temperature: 60.0
            motor_name = elem['motor_name']
            link_to_show = elem['link_to_show']
            self.marker_dict[motor_name] = elem
            self.marker_dict[motor_name]['marker_joint'] = self.make_marker(
                motor_name, link_to_show)
            self.marker_dict[motor_name]['marker_text'] = self.make_text_marker(
                motor_name, link_to_show)
            self.marker_dict[motor_name]['avg_tmp'] = 0.0
            rospy.loginfo(self.marker_dict[motor_name])

        self.diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray,
                                         self._diag_cb,
                                         queue_size=1)

        self.ma_pub = rospy.Publisher('temperature_markers', MarkerArray,
                                      queue_size=1)

    def _diag_cb(self, data):
        for st in data.status:
            for motor_name in self.marker_dict:
                if "Hardware: Motor: " + motor_name in st.name:
                    for value in st.values:
                        if value.key == 'Drive temperature':
                            val = float(value.value)
                            rospy.loginfo("Adjusting markers of " + str(motor_name))
                            self.adjust_markers(motor_name, val)
                            # if value.key == 'Motor temperature':

    def adjust_markers(self, motor_name, temperature):
        min_t = self.marker_dict[motor_name]['min_temperature']
        max_t = self.marker_dict[motor_name]['max_temperature']
        # The closer to min, the bluer, the closer to max, red
        rospy.loginfo("(temperature - min_t) / max_t:")
        rospy.loginfo(str(temperature) + " " + str(min_t) + " " + str(max_t))
        avg_tmp = (temperature - min_t) / max_t
        self.marker_dict[motor_name]['marker_joint'].color.r = avg_tmp
        self.marker_dict[motor_name]['marker_joint'].color.b = 1.0 - avg_tmp
        self.marker_dict[motor_name]['marker_joint'].id = self.marker_id
        self.marker_id += 1
        self.marker_dict[motor_name]['avg_tmp'] = avg_tmp

        self.marker_dict[motor_name]['marker_text'].text = motor_name + ': ' + str(temperature)
        self.marker_dict[motor_name]['marker_text'].color.r = avg_tmp
        self.marker_dict[motor_name]['marker_text'].color.b = 1.0 - avg_tmp
        self.marker_dict[motor_name]['marker_text'].id = self.marker_id
        self.marker_id += 1

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            ma = MarkerArray()
            for motor_name in self.marker_dict:
                if self.marker_dict[motor_name]['avg_tmp'] >= self.general_threshold:
                    ma.markers.append(self.marker_dict[motor_name]['marker_joint'])
                    ma.markers.append(self.marker_dict[motor_name]['marker_text'])
            self.ma_pub.publish(ma)
            r.sleep()

    def make_text_marker(self, motor_name, link_name):
        marker = Marker()
        marker.header.frame_id = link_name

        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.text = motor_name + ': XX.XXX'
        marker.ns = motor_name + '_text'
        marker.id = self.marker_id
        self.marker_id += 1
        return marker

    def make_marker(self, motor_name, link_name):
        marker = Marker()
        marker.header.frame_id = link_name
        # Automatically get mesh resource
        mesh_path, mesh_scale = get_link_mesh_info(link_name)
        if "package://" in mesh_path:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = str(mesh_path)
            if mesh_scale is not None:
                try:
                    marker.scale.x = float(mesh_scale.split()[0])
                    marker.scale.y = float(mesh_scale.split()[1])
                    marker.scale.z = float(mesh_scale.split()[2])
                except:
                    rospy.logwarn("Scale was not correctly found for " + str(self.base_name) +
                                  ", setting up 1.0 for all dimensions")
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0
            else:
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
        elif mesh_path == "CYLINDER":
            marker.type = Marker.CYLINDER
            marker.scale.x = float(mesh_scale["radius"])
            marker.scale.y = float(mesh_scale["radius"])
            marker.scale.z = float(mesh_scale["length"])
        elif mesh_path == "CUBE":
            marker.type = Marker.CUBE
            cube_size = mesh_scale["size"].split()
            marker.scale.x = float(cube_size[0])
            marker.scale.y = float(cube_size[1])
            marker.scale.z = float(cube_size[2])
        elif mesh_path == "SPHERE":
            marker.type = Marker.SPHERE
            marker.scale.x = float(mesh_scale["radius"])
            marker.scale.y = float(mesh_scale["radius"])
            marker.scale.z = float(mesh_scale["radius"])
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.ns = motor_name + '_mesh'
        marker.id = self.marker_id
        self.marker_id += 1

        return marker


if __name__ == '__main__':
    rospy.init_node('temperature_node')
    tm = TemperatureMarkers()
    tm.run()
