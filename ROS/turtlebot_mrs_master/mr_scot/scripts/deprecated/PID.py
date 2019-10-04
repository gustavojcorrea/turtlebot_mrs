#!/usr/bin/env python
# EE144 Lab4 closed-loop moving in a square
# May 8, 2018 by Hanzhe Teng, Keran Ye, Tianxiang Huang
import rospy
import tf

import smach
import smach_ros
import threading
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Point, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool, Float64, Int16
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from turtlebot_mrs_msgs.msg import RobotStatus
from kobuki_msgs.msg import *

import numpy as np
import matplotlib.pyplot as plt

from math import pi, sqrt, atan2
from scipy.interpolate import interp1d
from scipy import arange, array

class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D
