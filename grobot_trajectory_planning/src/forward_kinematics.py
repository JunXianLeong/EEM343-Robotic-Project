#!/usr/bin/env python

import numpy as np
from math import *

#link lengths for Arm 1(unit in m)
d1=0.30 
a2=1.20
a3=1.20
a4=0.33

#Angles for Arm 1
theta_1= 1.5880    #angles in degrees
theta_2= 2.1221
theta_3= -2.4000
theta_4= -0.5003

#Angles converted to radians
#theta_1=(theta_1/180)*pi
#theta_2=(theta_2/180)*pi
#theta_3=(theta_3/180)*pi
#theta_4=(theta_4/180)*pi
theta_234=theta_2+theta_3+theta_4

#FORWARD KINEMATICS for Arm 1
T_04=[[cos(theta_1)*cos(theta_234), -cos(theta_1)*sin(theta_234), sin(theta_1), cos(theta_1)*(a2*cos(theta_2)+a3*cos(theta_2+theta_3)+a4*cos(theta_234))],
      [sin(theta_1)*cos(theta_234), -sin(theta_1)*sin(theta_234), -cos(theta_1), sin(theta_1)*(a2*cos(theta_2)+a3*cos(theta_2+theta_3)+a4*cos(theta_234))],
      [sin(theta_234), cos(theta_234), 0, d1+ a2*sin(theta_2)+a3*sin(theta_2+theta_3)+a4*sin(theta_234)],
      [0, 0, 0, 1]]

print("T_04= ")
print(np.matrix(T_04))

#link lengths for Arm 2(unit in m)
L1=0.30
L2=1.20
L3=0.90
L4=0.1274
L5=0.1705
L6=0.0875

#Angles for Arm 2
Q1= 0.7940         #angles in degrees
Q2= 1.2767
Q3= -1.0868
Q4= -0.1898
Q5= -0.7250

#Angles converted to radians
#Q1=(Q1/180)*pi
#Q2=(Q2/180)*pi
#Q3=(Q3/180)*pi
#Q4=(Q4/180)*pi
Q234=Q2+Q3+Q4
Q23=Q2+Q3
#Q5=(Q5/180)*pi

#FORWARD KINEMATICS for Arm 2
X2=L2*cos(Q1)*cos(Q2)-L4*sin(Q1)-L5*cos(Q1)*sin(Q234)-L6*sin(Q1)*sin(Q5)+L3*cos(Q1)*cos(Q2)*cos(Q3)+L6*cos(Q1)*cos(Q234)*cos(Q5)-L3*cos(Q1)*sin(Q2)*sin(Q3)
Y2=L2*sin(Q1)*cos(Q2)+L4*cos(Q1)-L5*sin(Q1)*sin(Q234)+L6*cos(Q1)*sin(Q5)+L3*sin(Q1)*cos(Q2)*cos(Q3)+L6*sin(Q1)*cos(Q234)*cos(Q5)-L3*sin(Q1)*sin(Q2)*sin(Q3)
Z2=L1+L5*(cos(Q23)*cos(Q4)-sin(Q23)*sin(Q4))+L3*sin(Q23)+L2*sin(Q2)+L6*cos(Q5)*(cos(Q23)*sin(Q4)+sin(Q23)*cos(Q4))

T_05=[[cos(Q1)*cos(Q234)*cos(Q5)-sin(Q1)*sin(Q5), -sin(Q1)*cos(Q5)-cos(Q1)*cos(Q234)*sin(Q5), -cos(Q1)*sin(Q234), X2],
      [cos(Q1)*sin(Q5)+sin(Q1)*cos(Q234)*cos(Q5), cos(Q1)*cos(Q5)-sin(Q1)*cos(Q234)*sin(Q5), -sin(Q1)*sin(Q234), Y2],
      [sin(Q234)*cos(Q5), -sin(Q234)*sin(Q5), cos(Q234), Z2],
      [0, 0, 0, 1]]

print("T_05= ")
print(np.matrix(T_05))