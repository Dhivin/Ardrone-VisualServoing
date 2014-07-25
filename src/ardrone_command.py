#!/usr/bin/env python

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

class ArdroneCommand(object):
   def __init__(self,p=0.0,r=0.0,y=0.0,z=0.0):
      self.pitch = p
      self.roll  = r
      self.yaw   = y
      self.z     = z

   def getTwist(self):
      return Twist(Vector3(self.pitch,
                           self.roll,
                           self.z
                          ),
                   Vector3(0,
                           0,
                           self.yaw
                          )
                  )
