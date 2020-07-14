#!/usr/bin/env python

import numpy as np
from math import *

#link lengths for Arm 1(unit in m)
d1=0.30 
a2=1.20
a3=1.20
a4=0.33

#Angles for Arm 1
theta_1= 15.90755       #angles in degrees
theta_2= -3.1428376
theta_3= 65.785611
theta_4= -62.6427735

#Angles converted to radians
theta_1=(theta_1/180)*pi
theta_2=(theta_2/180)*pi
theta_3=(theta_3/180)*pi
theta_4=(theta_4/180)*pi

# DH Parameter Table for Arm 1
PT = [[theta_1, pi/2, 0, d1],
      [theta_2, 0, a2, 0],
      [theta_3, 0, a3, 0],
      [theta_4, 0, a4, 0]]

# Homogeneous Transformation Matrices
i = 0
H0_1 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
        [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
        [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
        [0, 0, 0, 1]]

i = 1
H1_2 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
        [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
        [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
        [0, 0, 0, 1]]

i = 2
H2_3 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
        [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
        [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
        [0, 0, 0, 1]]

i = 3
H3_4 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
        [sin(PT[i][0]), cos(PT[i][0])*cos(PT[i][1]), -cos(PT[i][0])*sin(PT[i][1]), PT[i][2]*sin(PT[i][0])],
        [0, sin(PT[i][1]), cos(PT[i][1]), PT[i][3]],
        [0, 0, 0, 1]]

print("H0_1 =")
print(np.matrix(H0_1))
print("H1_2 =")
print(np.matrix(H1_2))
print("H2_3 =")
print(np.matrix(H2_3))
print("H3_4 =")
print(np.matrix(H3_4))

H0_2 = np.dot(H0_1,H1_2)
H0_3 = np.dot(H0_2,H2_3)
H0_4 = np.dot(H0_3,H3_4)

print("H0_4 =")
print(np.matrix(H0_4))