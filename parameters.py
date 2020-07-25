import math
import numpy as np

#Structural details
arm_lengths = [1,1,1]
arm_angles = [math.pi/3,5*math.pi/3,math.pi]
motor_rots = [0,0,0]
center_mass = 1
motor_masses = [0.3,0.3,0.3]
tot_mass = center_mass+sum(motor_masses)
center = [0,0,0]
arm_height = 0
grav = 9.81
motor_pos = [[arm_lengths[i]*math.cos(arm_angles[i]),arm_lengths[i]*math.sin(arm_angles[i]),arm_height] for i in range(3)]
inertia = [[0 for i in range(3)] for j in range(3)]
for i in range(3):
    inertia[i][i] = sum(sum(motor_masses[k]*(motor_pos[k][(i+l)%3]-center[(i+l)%3])**2 for l in range(2)) for k in range(3))
    for j in (1,2):
        inertia[i][(i+j)%3] -= sum(motor_masses[k]*(motor_pos[k][i]-center[i])*(motor_pos[k][(i+j)%3]-center[(i+j)%3]) for k in range(3))
inertia = np.array(inertia)
inv_inertia = np.linalg.inv(inertia)
lin_resist = np.array([0,0,0])
ang_resist = np.array([0,0,0])
body_pts = [center] + motor_pos

#Electronics details
#Treats the battery-ESC-motor-prop assembly as one thing with one number for each motor
max_thrusts = [10,10,10]
max_torques = [0.2,0.2,0.2]
orientations = [1,-1,1]