#!/usr/bin/env python

import colorsys

# import ROS
import roslib
import rospy
#ROS msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def getARdroneMarker(frame_id):
   marker = Marker()
   marker.header.frame_id = frame_id
   marker.header.stamp = rospy.Time.now()
   marker.ns = "ARdroneMesh"
   marker.id = 0
   marker.type = 10
   marker.action = 0
   marker.lifetime = rospy.Duration(1.)
   marker.pose.position.x = 0.05
   marker.pose.position.y = 0.0
   marker.pose.position.z = -0.01
   marker.pose.orientation.x = 0.0
   marker.pose.orientation.y = 0.0
   marker.pose.orientation.z =  0.7071067811865476
   marker.pose.orientation.w = -0.7071067811865476
   marker.scale.x = 0.0006
   marker.scale.y = 0.0006
   marker.scale.z = 0.0006
   marker.color.a = 1.0
   marker.color.r = 0.4
   marker.color.g = 0.4
   marker.color.b = 0.4
   marker.mesh_resource = "package://ardrone_visual_servoing/res/ardrone2.stl";
   return marker

def getArMarker(frame_id, color, marker_id):
   marker = Marker()
   marker.header.frame_id = frame_id
   marker.header.stamp = rospy.Time.now()
   marker.ns = str(frame_id)+"_marker_"+str(marker_id)
   marker.id = marker_id
   marker.type = 1
   marker.action = 0
   marker.lifetime = rospy.Duration(1.)
   marker.pose.position.x = 0.0
   marker.pose.position.y = 0.0
   marker.pose.position.z = 0.0
   marker.pose.orientation.x = 0.0
   marker.pose.orientation.y = 0.0
   marker.pose.orientation.z =  0.0
   marker.pose.orientation.w = 1.0
   marker.scale.x = 0.106
   marker.scale.y = 0.106
   marker.scale.z = 0.0212
   (r,g,b) = colorsys.hsv_to_rgb(color, 1.0, 1.0)
   marker.color.a = 1.0
   marker.color.r = r
   marker.color.g = g
   marker.color.b = b
   return marker 
   
def getArrow(frame_id, color, p_start, p_end):
   marker = Marker()
   marker.header.frame_id = frame_id
   marker.header.stamp = rospy.Time.now()
   marker.ns = "arrow"
   marker.id = 0
   marker.type = 0
   marker.action = 0
   marker.lifetime = rospy.Duration(1.)
   marker.scale.x = 0.01
   marker.scale.y = 0.05
   marker.scale.z = 0.1
   (r,g,b) = colorsys.hsv_to_rgb(color, 1.0, 1.0)
   marker.color.a = 1.0
   marker.color.r = r
   marker.color.g = g
   marker.color.b = b
   marker.points = [p_start, p_end]
   return marker 

def getSphere(frame_id, color, p):
   marker = Marker()
   marker.header.frame_id = frame_id
   marker.header.stamp = rospy.Time.now()
   marker.ns = "sphere"
   marker.id = 0
   marker.type = 2
   marker.action = 0
   marker.lifetime = rospy.Duration(1.)
   marker.pose.position = p
   marker.pose.orientation.x = 0.0
   marker.pose.orientation.y = 0.0
   marker.pose.orientation.z =  0.0
   marker.pose.orientation.w = 1.0   
   marker.scale.x = 0.1
   marker.scale.y = 0.1
   marker.scale.z = 0.1
   (r,g,b) = colorsys.hsv_to_rgb(color, 1.0, 1.0)
   marker.color.a = 1.0
   marker.color.r = r
   marker.color.g = g
   marker.color.b = b
   return marker

