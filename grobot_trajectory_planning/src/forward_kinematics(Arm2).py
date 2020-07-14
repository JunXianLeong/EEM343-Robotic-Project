#!/usr/bin/env python

import numpy as np
from math import *

#link lengths for Arm 2(unit in m)
L1=0.30
L2=1.20
L3=0.90
L4=0.1274
L5=0.1705
L6=0.0875

#Angles for Arm 2
Q1= 0         #angles in degrees
Q2= 0
Q3= 0
Q4= 0
Q5= 180

#Angles converted to radians
Q1=(Q1/180)*pi
Q2=(Q2/180)*pi
Q3=(Q3/180)*pi
Q4=(Q4/180)*pi
Q234=Q2+Q3+Q4
Q23=Q2+Q3
Q5=(Q5/180)*pi

# DH Parameter Table for Arm 1
PT = [[Q1, pi/2, 0, L1],
      [Q2, 0, L2, 0],
      [Q3, 0, L3, -L4],
      [Q4, -pi/2, 0, 0],
      [Q5, 0, L6, L5]]

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

i = 4
H4_5 = [[cos(PT[i][0]), -sin(PT[i][0])*cos(PT[i][1]), sin(PT[i][0])*sin(PT[i][1]), PT[i][2]*cos(PT[i][0])],
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
print("H4_5 =")
print(np.matrix(H4_5))

H0_2 = np.dot(H0_1,H1_2)
H0_3 = np.dot(H0_2,H2_3)
H0_4 = np.dot(H0_3,H3_4)
H0_5 = np.dot(H0_4,H4_5)

print("H0_5 =")
print(np.matrix(H0_5))