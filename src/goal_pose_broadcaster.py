#!/usr/bin/env python

import roslib
import rospy

import tf

from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from ardrone_visual_servoing.msg import GoalMarker

PUBLISH_FRQ = 10.0 # [Hz]

class GoalPoseBroadcaster(object):
   def __init__(self):    
      rospy.loginfo('Initializing goal pose broadcaster')
      
      '''
      init variables
      '''
      # goal marker frame
      self.goalMarkerFrame = "ar_marker_6"
      # goal pose
      p = Point(0.,0.,1.5)
      q = Quaternion(0.,0.,0.,1.)
      initPose = Pose(p,q)
      self.goalPose = initPose
      # goal velocity
      v_zero = Vector3(0.,0.,0.)
      initVel = Twist(v_zero,v_zero)
      self.goalVelocity = initVel
      
      '''
      init TF
      '''
      # TF listener
      #self.tfl = tf.TransformListener()
      # TF broadcaster
      self.tfb = tf.TransformBroadcaster()
      #TF broadcaster callback timer
      self.timer = rospy.Timer(rospy.Duration(1. / PUBLISH_FRQ), self.tfBroadcasterCallback) 

      '''
      init subscribers
      '''
      self.subGoalPose = rospy.Subscriber('/ardrone_visual_servoing/goal_pose', Pose, self.receiveGoalPose)
      self.subGoalMarkerFrame = rospy.Subscriber('/ardrone_visual_servoing/goal_marker_frame',GoalMarker,self.receiveGoalMarkerFrame)

   def receiveGoalMarkerFrame(self,msg):
      self.goalMarkerFrame = msg.goal_marker_frame

   def receiveGoalPose(self,msg):
      self.goalPose = msg

   def getGoalPose(self):
      return self.goalPose

   def getGoalVelocity(self):
      return self.goalVelocity
   
   def tfBroadcasterCallback(self,event):
      p = self.goalPose.position
      o = self.goalPose.orientation
      self.tfb.sendTransform((p.x, p.y, p.z),
                             (o.x, o.y, o.z, o.w),
                             rospy.Time.now(),
                             "ardrone_goal_pose",
                             self.goalMarkerFrame
                            )
