from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import pyquaternion
import actuators as a
import parameters as p
import numpy as np
import math

#PDI
#NOT PID
#Controller coefficients for each kind of motion
K_dir = np.array((0.3,0.9,0.001))
K_yaw = np.array((0.5,4,0.001))
K_vert = np.array((0.02,0.01,0.004))
K_horiz = np.array((0.04,0.03,0.0004))

#Limits any angle to the range (-pi,pi]
def restrict(angle):
    x = angle % (math.pi*2)
    if x > math.pi:
        x -= 2*math.pi
    return x

#Does some vector algebra to make something that kinda aligns with error
def curr_error(pos,acc,desired_dir,desired_ang,desired_acc):
    error_vec = -np.cross(np.cross(pos.rotate(np.array(desired_dir)), (0,0,1)), desired_dir)
    test_vec = np.array((1,0,0))
    permuted = pos.rotate(test_vec)

    world_acc = pos.conjugate.rotate(acc) + np.array((0,0,-p.grav))

    return pos.conjugate.rotate(error_vec),\
           restrict(math.atan2(permuted[1],permuted[0])-math.atan2(test_vec[1],test_vec[0])-desired_ang),\
           desired_acc - world_acc

#Gets error into a PID format
def effective_error(prev_err,current):
    return [(current[i],current[i]-prev_err[i][0],prev_err[i][2]+current[i]) for i in range(3)]

#Does all the PID loops
def impulse_change(state,errors):
    acc_error = np.transpose(errors[2])
    vert_scaling = K_vert.dot(acc_error[2])

    thrust_goal = K_dir.dot(errors[0])
    thrusts = [0,0,0]
    for i in range(3):
        thrusts[i] = thrust_goal.dot((p.motor_pos[i])) + vert_scaling

    angle = K_yaw.dot(errors[1])
    dir_goal = np.array((K_horiz.dot(acc_error[0]),K_horiz.dot(acc_error[1]),0))
    dir_goal[2] = math.sqrt(1 - np.linalg.norm(dir_goal)**2)

    return np.array(thrusts),angle,dir_goal