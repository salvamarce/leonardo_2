#!/usr/bin/env python

import roslib
from visualization_msgs.msg import Marker
import rospy
import math

import time

if __name__ == '__main__':
   try:
      topic = 'map_visualization'
      pub = rospy.Publisher(topic, Marker, queue_size=1)

      rospy.init_node('publish_map')

      while not rospy.is_shutdown():

         marker = Marker()
         marker.header.frame_id = "map"
         marker.type = marker.MESH_RESOURCE
         marker.mesh_resource = "package://leonardo_2/models/campo_gara/meshes/campo_gara.dae"
         marker.scale.x = 1.0
         marker.scale.y = 1.0
         marker.scale.z = 1.0
         marker.pose.orientation.w = 1.0
         marker.pose.position.x = 0.0
         marker.pose.position.y = 0.0
         marker.pose.position.z = 0.0
         marker.color.a = 1.0;
         marker.color.r = 0.5;
         marker.color.g = 0.5;
         marker.color.b = 0.5;

         pub.publish(marker)
   
   except rospy.ROSInterruptException:
      pass

