#!/usr/bin/env python

import roslib
import rospy

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from ardrone_autonomy.msg import Navdata

from ardrone_status import ARdroneStatus
from ardrone_command import ArdroneCommand

class ARdroneDriver(object):
   def __init__(self):
      rospy.loginfo('Initializing AR.Drone driver')

      # define current AR.drone status 
      self.status  = -1
      
      # subscribe to /ardrone/navdata topic.
      self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.receiveNavdata)  
   
      # define current command
      self.command = Twist()
   
      # lets the controller publish movement command to the ARdrone
      self.pubCommand = rospy.Publisher('/cmd_vel', Twist)

      # lets the controller publicsh takeOff, land & reset commands to the ARdrone
      self.pubTakeoff  = rospy.Publisher('/ardrone/takeoff', Empty)
      self.pubLand     = rospy.Publisher('/ardrone/land', Empty)
      self.pubReset    = rospy.Publisher('/ardrone/reset', Empty)

      # land the ARdrone if the node is shutdown
      rospy.on_shutdown(self.land)

   def receiveNavdata(self, navdata):
      # read the state of the ARdrone
      self.status = navdata.state

   def velocityCommand(self, cmd):
      self.command = Twist(Vector3(cmd.pitch,
                                   cmd.roll,
                                   cmd.z
                                  ),
                           Vector3(0,
                                   0,
                                   cmd.yaw
                                  )
                          )
      if (self.status == ARdroneStatus.Flying or self.status == ARdroneStatus.GotoHover or self.status == ARdroneStatus.Hovering):
         self.pubCommand.publish(self.command)

   def setVelocity(self, pitch=0., roll=0., yaw=0., height=0.):
      self.command = Twist(Vector3(pitch,roll,height),Vector3(0,0,yaw))
      if (self.status == ARdroneStatus.Flying or self.status == ARdroneStatus.GotoHover or self.status == ARdroneStatus.Hovering):
         self.pubCommand.publish(self.command)
   
   def hower(self):
      self.command = Twist(Vector3(0,0,0),Vector3(0,0,0))

   def takeOff(self):
      rospy.loginfo("AR.Drone cmd: Take off")
      if (self.status == ARdroneStatus.Landed):
         self.pubTakeoff.publish(Empty())

   def land(self):
      rospy.loginfo("AR.Drone cmd: Landing")
      self.pubLand.publish(Empty())
      
   def emergencyStop(self):
      rospy.loginfo("AR.Drone cmd: Emergency stop")
      self.pubReset.publish(Empty())
