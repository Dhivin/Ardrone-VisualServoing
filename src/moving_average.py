#!/usr/bin/env python

from collections import deque
import math

class movingAverage(object):
   def __init__(self, size=10):
      self.maxSize = size
      self.array = deque([])

   def setLength(self, length):
      self.maxSize = length
      self.clear()
      
   def append(self, i):
      if len(self.array) < self.maxSize:
         self.array.extend([i])
      else:
         self.array.popleft()
         self.array.append(i)
      return self.average()

   def average(self):
      res = 0.0
      if len(self.array) != 0:
         res = sum(self.array) / len(self.array)
      return res

   def clear(self):
      self.array.clear()


