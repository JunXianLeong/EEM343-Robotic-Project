#!/usr/bin/env python

import sys
from math import *

#Desired position of End effector for Arm 1
px=2.0
py=0.57
pz=1.3

#Desired orientation of End effector for Arm 1
#phi=theta2+theta3+theta4
phi=0

#Inverse Kinematics equation for Arm 1
theta_1= atan2(py,px)
q3y=((px*cos(theta_1)+py*sin(theta_1)-0.33*cos(phi))**2+(pz-0.3-0.33*sin(phi))**2-2.88)
q3x=2.88
theta_3= -acos(q3y/q3x)
q2y=(1.2*cos(theta_3)+1.2)*(pz-0.3-0.33*sin(phi))-1.2*sin(theta_3)*(px*cos(theta_1)+py*sin(theta_1)-0.33*cos(phi))
q2x=(1.2*cos(theta_3)+1.2)*(px*cos(theta_1)+py*sin(theta_1)-0.33*cos(phi))+1.2*sin(theta_3)*(pz-0.3-0.33*sin(phi))
theta_2= atan2(q2y,q2x)
theta_4= phi-theta_2-theta_3

print("theta_1(rad)= ")
print(theta_1)

print("theta_2(rad)= ")
print(theta_2)

print("theta_3(rad)= ")
print(theta_3)

print("theta_4(rad)= ")
print(theta_4)

#convert from rad to degrees
theta_1=(theta_1/pi)*180
theta_2=(theta_2/pi)*180
theta_3=(theta_3/pi)*180
theta_4=(theta_4/pi)*180

print("theta_1(deg)= ")
print(theta_1)

print("theta_2(deg)= ")
print(theta_2)

print("theta_3(deg)= ")
print(theta_3)

print("theta_4(deg)= ")
print(theta_4)

#Desired position of End effector for Arm 2
px=1.58
py=0.57
pz=1.3

#Desired orientation of End effector for Arm 2
az=0
nz=1

#phi=theta2+theta3+theta4
phi_2=0

#Inverse Kinematics equation for Arm 2
Q5= atan(-az/nz)
c= 0.0875*sin(Q5)-0.1274
r=((2*c*py)**2)-4*((py)**2+(px)**2)*(-(c)**2-(px)**2)
q11= (-2*c*py+sqrt(r))/(2*(py**2+px**2))
Q1= acos(q11)
Q3y=(px*cos(Q1)+py*sin(Q1)-0.0875*cos(phi_2)*cos(Q5)+0.1705*sin(phi_2))**2+(pz-0.3-0.0875*sin(phi_2)*cos(Q5)-0.1705*cos(phi_2))**2-2.25
Q3x=2.16
Q3= -acos(Q3y/Q3x)
Q2y=(0.9*cos(Q3)+1.2)*(pz-0.3-0.0875*sin(phi_2)*cos(Q5)-0.1705*cos(phi_2))-0.9*sin(Q3)*(px*cos(Q1)+py*sin(Q1)-0.0875*cos(phi_2)*cos(Q5)+0.1705*sin(phi_2))
Q2x=(0.9*cos(Q3)+1.2)*(px*cos(Q1)+py*sin(Q1)-0.0875*cos(phi_2)*cos(Q5)+0.1705*sin(phi_2))+0.9*sin(Q3)*(pz-0.3-0.0875*sin(phi_2)*cos(Q5)-0.1705*cos(phi_2))
Q2= atan2(Q2y,Q2x)
Q4= phi_2-Q2-Q3

print("Arm 2: ")
print("Q1(rad)= ")
print(Q1)

print("Q2(rad)= ")
print(Q2)

print("Q3(rad)= ")
print(Q3)

print("Q4(rad)= ")
print(Q4)

print("Q5(rad)= ")
print(Q5)

#convert from rad to degrees
Q1=(Q1/pi)*180
Q2=(Q2/pi)*180
Q3=(Q3/pi)*180
Q4=(Q4/pi)*180
Q5=(Q5/pi)*180

print("Q1(deg)= ")
print(Q1)

print("Q2(deg)= ")
print(Q2)

print("Q3(deg)= ")
print(Q3)

print("Q4(deg)= ")
print(Q4)

print("Q5(deg)= ")
print(Q5)