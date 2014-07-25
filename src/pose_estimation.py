#!/usr/bin/env python

import sys

# import libs
import numpy as np
import math
import PyKDL


# import ROS
import roslib
import rospy

# ROS libs
import tf
import tf.transformations
from tf_conversions import posemath

# ROS msgs
import std_msgs.msg
from std_msgs.msg import String
import sensor_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Vector3
import ar_track_alvar.msg
import ardrone_autonomy.msg
import visualization_msgs.msg
from visualization_msgs.msg import Marker
from ar_track_alvar.msg import AlvarMarker
from ar_track_alvar.msg import AlvarMarkers
from ardrone_visual_servoing.msg import GoalMarker

# import 
from moving_average import movingAverage

# constants
pi = math.pi

class ARdronePoseEstimation(object):
   def __init__(self):
      rospy.loginfo('Initializing pose estimation')

      '''
      init variables
      '''
      # goal marker
      self.goalMarkerId    = 6
      self.goalMarkerFrame = "ar_marker_"+str(self.goalMarkerId)
      # init moving average filters
      self.ma_x = movingAverage()
      self.ma_y = movingAverage()
      self.ma_z = movingAverage()

      '''
      Init subscribers
      '''
      self.subArMarkerPose = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.receiveArMarkerPose)
      self.subGoalMarkerFrame = rospy.Subscriber('/ardrone_visual_servoing/goal_marker_frame',GoalMarker,self.receiveGoalMarkerFrame)      

      '''
      init publishers
      '''

      '''
      init TF
      '''
      # TF listener
      self.tfl = tf.TransformListener()
      # TF broadcaster
      self.tfb = tf.TransformBroadcaster()

   def setLowpassLength(self,length):
      self.ma_x.setLength(length)
      self.ma_y.setLength(length)
      self.ma_z.setLength(length)

   def receiveGoalMarkerFrame(self,msg):
      self.goalMarkerId    = msg.goal_marker_id
      self.goalMarkerFrame = msg.goal_marker_frame

   def receiveArMarkerPose(self,msg):
      for m in msg.markers:
         if m.id == self.goalMarkerId:
            p = m.pose.pose.position
            o = m.pose.pose.orientation
 
            # convert msg into PyKDL
            pos = PyKDL.Vector(p.x,p.y,p.z)
            rot = PyKDL.Rotation.Quaternion(o.x,o.y,o.z,o.w)
            f_alvar = PyKDL.Frame(rot,pos)
            
            # inverse coordinate
            f_inv = f_alvar.Inverse()
            #fix rotation error
            f_rot = PyKDL.Frame(PyKDL.Rotation.RPY(pi,0,-pi/2),PyKDL.Vector(0.,0.,0.))
            f_drone_est  = f_inv * f_rot
            # create planar est
            (r,p,y) = f_drone_est.M.GetRPY()
            #r_rot_inv = PyKDL.Frame(PyKDL.Rotation.RPY(r,p,0.),PyKDL.Vector(0.,0.,0.))
            #f_drone_planar = f_drone_est * r_rot_inv.Inverse()
            f_drone_planar = PyKDL.Frame(PyKDL.Rotation.RPY(0.,0.,y),PyKDL.Vector(*f_drone_est.p))
            # convert back to ROS
            pose = posemath.toMsg(f_drone_est)
            p = pose.position
            o = pose.orientation

            self.tfb.sendTransform( (p.x,p.y,p.z),
                                    (o.x,o.y,o.z,o.w),
                                    rospy.Time.now(),
                                    "ARdrone_est",
                                    self.goalMarkerFrame)
            
            lp_x = self.ma_x.append(p.x)
            lp_y = self.ma_y.append(p.y)
            lp_z = self.ma_z.append(p.z)
            self.tfb.sendTransform( (lp_x,lp_y,lp_z),
                                    (o.x,o.y,o.z,o.w),
                                    rospy.Time.now(),
                                    "ARdrone_est_lp",
                                    self.goalMarkerFrame)            

            pose = posemath.toMsg(f_drone_planar)
            p = pose.position
            o = pose.orientation
            self.tfb.sendTransform( (p.x,p.y,p.z),
                                    (o.x,o.y,o.z,o.w),
                                    rospy.Time.now(),
                                    "ARdrone_est_planar",
                                    self.goalMarkerFrame)

            self.tfb.sendTransform( (lp_x,lp_y,lp_z),
                                    (o.x,o.y,o.z,o.w),
                                    rospy.Time.now(),
                                    "ARdrone_est_lp_planar",
                                    self.goalMarkerFrame)
                  
if __name__ == '__main__':
   try:
      ARdronePoseEstimation()
   except rospy.ROSInterruptException: pass
