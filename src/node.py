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
from tf.transformations import euler_from_quaternion

#ROS msgs
from std_msgs.msg import Header
from ardrone_autonomy.msg import Navdata
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from ar_track_alvar.msg import AlvarMarker
from ar_track_alvar.msg import AlvarMarkers
from ardrone_visual_servoing.msg import CtrlState
from ardrone_visual_servoing.msg import ARdroneControllerState
from ardrone_visual_servoing.msg import PoseEstimator


# import
from ardrone_driver import ARdroneDriver
from pose_estimation import ARdronePoseEstimation
from goal_pose_broadcaster import GoalPoseBroadcaster
from ardrone_command import ArdroneCommand
import joystick_mapping as jmap

# Constants
#control
CONTROL_FREQ = 50    # [Hz]
#controller states
STATE_MAUNAL_CONTROL = "Manual control"
STATE_AUTO_PILOT = "Auto pilot"
# auto pilot state
AUTO_PILOT_ACTIVE = "active control"
AUTO_PILOT_HOVER  = "hovering"
# timing
POSE_EXPIRE_DURATION     = 1.0
VELOCITY_EXPIRE_DURATION = 0.2

class ARdroneVisualServoingController(object):
   def __init__(self):
      rospy.init_node('ardrone_visual_servoing_controller')
      rospy.loginfo('AR.Drone Visual Servoing Node started')

      # AR.Drone driver
      self.driver = ARdroneDriver()
      # AR.Drone pose estimation
      self.poseEst = ARdronePoseEstimation()
      # Goal pose broadcaster
      self.goalPoseBroadcaster = GoalPoseBroadcaster()

      '''
      init subscribers
      '''
      # joystick
      self.subJoy = rospy.Subscriber('/joy', Joy, self.receiveJoyMsg)
      # AR.drone Navdata
      self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.receiveNavdata)
      # PID gains
      self.subPid = rospy.Subscriber('/ardrone_visual_servoing/pid',Vector3,self.receivePid)
      # pose estimation
      self.subPoseEst = rospy.Subscriber('/ardrone_visual_servoing/pose_est',PoseEstimator,self.receivePoseEstSettings)

      '''
      init publisher
      '''
      # controller state
      self.pubControllerState = rospy.Publisher('/ardrone_visual_servoing/controller_state', CtrlState)

      '''
      init TF
      '''
      # TF listener
      self.tfl = tf.TransformListener()

      '''
      init variables
      '''
      # controller state
      self.autoPilotState = False
      # AR.Drone velocity 
      self.vel = np.zeros((3,1))
      self.velTimeStamp = rospy.Time(0)
      # pose estimation
      self.activePoseEstFrame = "ARdrone_est_planar"
      # error
      self.error_p = np.zeros((3,1))
      self.error_i = np.zeros((3,1))
      self.error_d = np.zeros((3,1))
      # commands
      cmd_zero = ArdroneCommand()
      self.cmd_manual    = cmd_zero
      self.cmd_autopilot = cmd_zero
      # auto pilot
      self.autoPilotActive = False
      # manual control cmd from joystick
      j = Joy()
      j.axes    = [0.0,0.0,0.0,0.0,0.0,0.0]
      j.buttons = [0,0,0,0,0,0,0,0,0,0,0,0]
      self.joyCmd = j
      # pid
      self.Kp = 0.8
      self.Ki = 0.0
      self.Kd = 0.0

      '''
      control loop
      '''
      # set control loop frequency
      self.r = rospy.Rate(CONTROL_FREQ)
      # control loop
      while not rospy.is_shutdown():
         # calculate errors (pos & vel)
         self.calcErrors()

         # gen drone cmd (man and auto)
         self.cmd_manual    = self.manualControl()
         self.cmd_autopilot = self.autoPilot()

         # select cmd based on state and send cmd to driver
         if self.autoPilotState == True:
            self.driver.velocityCommand(self.cmd_autopilot)
         else:
            self.driver.velocityCommand(self.cmd_manual)
         
         #publish controller state msg
         self.pubControllerState.publish(self.generateCtrlStateMsg())

         self.r.sleep()

      rospy.signal_shutdown("ARdroneVisualServoingController shutdown")

   def generateCtrlStateMsg(self):
         stateMsg = CtrlState()
         h = Header()
         h.stamp = rospy.Time.now()
         h.frame_id = ""
         stateMsg.header = h

         stateMsg.controller_state = STATE_AUTO_PILOT  if (self.autoPilotState)  else STATE_MAUNAL_CONTROL
         stateMsg.autopilot_state  = AUTO_PILOT_ACTIVE if (self.autoPilotActive) else AUTO_PILOT_HOVER

         stateMsg.goal_pos  = self.goalPoseBroadcaster.getGoalPose()

         if self.error_p != None:
            stateMsg.calc_error = True
            stateMsg.pos_error = Pose(Point(*self.error_p),Quaternion(0.,0.,0.,1.))
         else:
            stateMsg.calc_error = False

         stateMsg.cmd_manual    = self.cmd_manual.getTwist()
         stateMsg.cmd_autopilot = self.cmd_autopilot.getTwist()

         stateMsg.pid = Vector3(self.Kp, self.Ki, self.Kd)

         return stateMsg

   def calcErrors(self):
      f_from = 'ardrone_goal_pose'
      f_to   = self.activePoseEstFrame
      # calculate proportional error
      try:
         common_time = self.tfl.getLatestCommonTime(f_from, f_to)
         now = rospy.Time.now()
         if common_time != 0 and (now - common_time) < rospy.Duration(POSE_EXPIRE_DURATION):
            (p,o) = self.tfl.lookupTransform(f_from,f_to, rospy.Time(0))

            #calc error vector
            rot = PyKDL.Rotation.Quaternion(*o)
            pos = PyKDL.Vector(*p)

            f_goal  = PyKDL.Frame(rot,pos)
            f_rot   = PyKDL.Frame(rot,PyKDL.Vector(0.,0.,0.))
            f_error = f_goal.Inverse() * f_rot

            p_error = Point(*f_error.p)
         
            self.error_p = np.array([[p_error.x,
                                      p_error.y,
                                      p_error.z
                                    ]]).T
         else:
            self.error_p = None
      except (tf.Exception, tf.LookupException, tf.ConnectivityException):
         self.error_p = None

      #calculate integral error
      ### Not used at the moment
      ### self.error_i = np.zeros((3,1))

      # calculate derivative error
      now = rospy.Time.now()
      if self.velTimeStamp != 0 and (now - self.velTimeStamp) < rospy.Duration(VELOCITY_EXPIRE_DURATION):
         v_goal = self.goalPoseBroadcaster.getGoalVelocity()
         v_des = np.array([[v_goal.linear.x, 
                            v_goal.linear.y, 
                            v_goal.linear.z
                          ]]).T
         self.error_d = v_des - self.vel
      else:
         self.error_d = None

   def manualControl(self):
      pitch  = self.joyCmd.axes[jmap.pitch_axis]
      roll   = self.joyCmd.axes[jmap.roll_axis]
      yaw    = self.joyCmd.axes[jmap.yaw_axis]
      height = self.joyCmd.axes[jmap.height_axis]
      return ArdroneCommand(pitch,roll,yaw,height)

   def autoPilot(self):
      # PID Controller
      if (self.error_p != None) and (self.error_i != None or self.Ki == 0.0) and (self.error_d != None or self.Kd == 0.0):
         u_p = self.Kp * self.error_p
         u_i = self.Ki * self.error_i if (self.Ki != 0.0) else 0.0
         u_d = self.Kd * self.error_d if (self.Kd != 0.0) else 0.0
         
         u = u_p + u_i + u_d
         self.autoPilotActive = True
      else:
         u = np.zeros((3,1))
         self.autoPilotActive = False

      pitch = u[0]
      roll  = u[1]
      yaw   =  0.0
      z     = u[2]

      return ArdroneCommand(pitch,roll,yaw,z)

   def receivePoseEstSettings(self,msg):
      self.activePoseEstFrame = msg.pose_est_frame + "_planar"
      self.poseEst.setLowpassLength(msg.lowpass_length)

   def receiveJoyMsg(self,msg):
      self.joyCmd = msg
      # check for controller state transitions
      if self.joyCmd.buttons[jmap.emergency_btn] == 1:
         self.autoPilotState = False
         self.driver.emergencyStop()
      elif self.joyCmd.buttons[jmap.landing_btn] == 1:
         self.autoPilotState = False
         self.driver.land()
      elif self.joyCmd.buttons[jmap.takeoff_btn] == 1:
         self.autoPilotState = False
         self.driver.takeOff()
      elif any(p != 0.0 for p in self.joyCmd.axes):
         self.autoPilotState = False
      elif self.joyCmd.buttons[jmap.autopilot_btn] == 1:
         self.autoPilotState = True
   
   def receiveNavdata(self,msg):
      # AR.Drone velocity
      self.vel = np.array([[msg.vx/1000.0, #scale from [mm/s] to [m/s]
                            msg.vy/1000.0, 
                            msg.vz/1000.0
                          ]]).T
      self.velTimeStamp = msg.header.stamp 

   def receivePid(self,msg):
      self.Kp = msg.x
      self.Ki = msg.y
      self.Kd = msg.z

# initialization
if __name__ == '__main__':
   try:
      ARdroneVisualServoingController()
   except rospy.ROSInterruptException: pass     
