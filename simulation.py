from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import pyquaternion
import actuators as a
import parameters as p
import numpy as np
import math
import display as d
import controls as c
import random
import time
import os
from pynput.keyboard import Listener, KeyCode

#Moves the pygame window to be at the top left of the monitor
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (-100,-100)
# time.sleep(2)

#I ran into some bug with drag so instead of fixing it I just set drag to 0
#So this currently doesn't do anything
def drag_2(state):
    lin_drag = state[2].conjugate.rotate((state[2].rotate(state[1])) * p.lin_resist*-1)
    rot_drag = state[2].conjugate.rotate(state[2].rotate(state[3]) * p.ang_resist*-1)
    return lin_drag,rot_drag

#Todo: UNWRAP THESE FUNCTIONS LATER SINCE THEYRE NOT USEFUL
#Otherwise just calculates 2nd derivatives for q and position
def RK4_helper(pos,vel,q,q_dot,powers,angles,dt):
    thrust_forces = a.full_thrust(powers, angles)
    omega = (2*q.conjugate * q_dot).vector
    drag_forces = drag_2((pos,vel,q,omega))
    thrust = q.rotate((thrust_forces[0]+drag_forces[0])/p.tot_mass)
    thrust[2] -= p.grav
    body_rot = (q.conjugate * q_dot).vector
    q_acc = (q_dot*q.conjugate*q_dot + 1/2*q*pyquaternion.Quaternion(
        vector=p.inv_inertia.dot(thrust_forces[1]+drag_forces[1]-4*np.cross(body_rot,p.inertia.dot(body_rot)))))
    return thrust,q_acc

#Does RK4 on position and rotation independently
def RK4_step(state,powers,angles,dt):
    q_dot_base = 1/2 * pyquaternion.Quaternion(vector=state[3]) * state[2]
    index = [(0,0),(0,0),(0,1),(2,2)]
    coeff = [0,1/2,1/2,1]
    k_t = [0.0,0.0,0.0,0.0]
    k_r = [0.0,0.0,0.0,0.0]
    for i in range(4):
        k_t[i] = RK4_helper(state[0]+coeff[i]*dt*state[1]+(coeff[i]*dt)**2/2*k_t[index[i][0]],
                            state[1]+coeff[i]*dt*k_t[index[i][1]],state[2],q_dot_base,powers,angles,dt)[0]
        k_r[i] = RK4_helper(state[0],state[1],state[2]+coeff[i]*dt*q_dot_base+(coeff[i]*dt)**2/2*k_r[index[i][0]],
                            q_dot_base+coeff[i]*dt*k_r[index[i][1]],powers,angles,dt)[1]
    updated = state.copy()
    updated[0] = state[0]+dt*state[1]+dt**2/6*(k_t[0]+k_t[1]+k_t[2])
    updated[1] = state[1]+dt/6*(k_t[0]+2*k_t[1]+2*k_t[2]+k_t[3])
    updated[2] = (state[2]+dt*q_dot_base+dt**2/6*(k_r[0]+k_r[1]+k_r[2])).unit
    q_dot_new = q_dot_base+dt/6*(k_r[0]+2*k_r[1]+2*k_r[2]+k_r[3])
    updated[3] = (2*q_dot_new*updated[2].conjugate).vector

    return updated

#Updates the internal model for the tricopter's position
#Only takes in the current angular velocity, like a real gyro would give
def internal_model(rotation,angvel,dt):
    q_dot_start = 1/2*pyquaternion.Quaternion(vector=angvel) * rotation
    return (rotation + dt*q_dot_start).unit

#Handles the keyboard input stuff with pynput
keys_pressed = set()
def on_press(key):
    keys_pressed.add(eval(str(key)))
def on_release(key):
    keys_pressed.remove(eval(str(key)))
listener =  Listener(
        on_press=on_press,
        on_release=on_release)
listener.start()

def handle_input(keys):
    out1 = [0,0,0]
    out2 = 0
    if 'w' in keys:
        out1[0] += 3
    if 'a' in keys:
        out1[1] += 3
    if 's' in keys:
        out1[0] -= 3
    if 'd' in keys:
        out1[1] -= 3
    if 'q' in keys:
        out2 += 0.08
    if 'e' in keys:
        out2 += -0.08
    return out1,out2

#Many many initializations sorry
error = [(0,0,0) for i in range(3)]
n = 3000
pts = [[[0 for i in range(n)] for j in range(3)] for k in range(4)]

#State: [[pos],[velocity],[current rotation],[angular velocity]]
state = [[],[],[],[]]
state[0] = np.array([0.0,0.0,0.0])
state[1] = np.array([0.0,0.0,0.0])
# state[2] = np.eye(3)
state[2] = pyquaternion.Quaternion((1,0,0,0)).unit
state[3] = np.array([0,0,0])

#What the tricopter thinks its rotation is
int_model = pyquaternion.Quaternion((1,0,0,0)).unit

#Useful for updating the internal model
prev_rot = state[3].copy()

#The default thrust values for each motor
base_thrust = np.array((0.63,0.63,0.63))

#Don't change this specifically, the control algorithm updates this
# by the expected horizontal velocity
dir_goal = np.array((0.1,0,1))
dir_goal = dir_goal/np.linalg.norm(dir_goal)

#Used to determine desired rotation about the z-axis
angoal = 0
angvel = 0
prev = state[3].copy()

#Changes that the control algorithm makes to the motors
adj_thrust = (0,0,0)
adj_angle = 0

#Desired acceleration vector
acc_goal = [0,0,0]

#How many RK4 steps happen per internal step
step_ratio = 2
dt = 0.1/step_ratio

for i in range(n):

    #Gets position for the final plot at the end
    for k in range(4):
        pos = state[2].rotate(np.array(p.body_pts[k])) + state[0]
        # pos = state[2].dot(np.array(p.body_pts[k])) + state[0]
        # print(pos,state[1],state[3])
        for j in range(3):
            pts[k][j][i] = pos[j]

    #Performs internal control calculations
    if i % step_ratio == 0:

        #Does the control algorithm
        curr_acc = a.full_thrust(base_thrust+adj_thrust,[0,0,0.02+max(-0.06,min(0.06,adj_angle))])[0] / p.tot_mass
        err = c.curr_error(int_model,curr_acc,dir_goal,angoal,acc_goal)
        error = c.effective_error(error,err)
        adj_thrust,adj_angle,dir_goal = c.impulse_change(state,error)

        #Updates internal model and gets keyboard input
        int_model = internal_model(int_model,2/2*(0*prev_rot+state[3]),dt*step_ratio)
        acc_goal,angvel = handle_input(keys_pressed)
        angoal += angvel/step_ratio
        prev_rot = state[3]

    #Does an RK4 step
    state = RK4_step(state,base_thrust+adj_thrust,[0,0,0.02+max(-0.06,min(0.06,adj_angle))],dt)

    #Prints debug info every so often
    if i % 20 == 0:
        # print(dir_goal)
        print((state[2] - int_model).norm, '\t')
        # print(state[3],np.linalg.norm(state[3]),np.linalg.norm(p.inertia.dot(state[3])),np.linalg.norm(p.inertia.dot(p.inertia).dot(state[3])))

    #Displays a frame of movement with pygame
    d.disp_frame(state)



#Prints stuff at the end
print()
for e in error:
    print(e)
print()
for s in state:
    print(s)

#Shows the final map of the tricopter's trajectory
ax = plt.axes(projection='3d')
ax.scatter3D(pts[0][0], pts[0][1], pts[0][2],c='#111199')
ax.scatter3D(pts[1][0], pts[1][1], pts[1][2],c='#119911')
ax.scatter3D(pts[2][0], pts[2][1], pts[2][2],c='#119911')
ax.scatter3D(pts[3][0], pts[3][1], pts[3][2],c='#991111')
plt.show()
