import parameters as p
import math
import numpy as np

#Finds the forces and torques due to a single motor
#Relative to the center of mass of the tricopter
#NOT absolute directions
def single_motor(index, power, angle):
    direction = [math.sin(angle)*math.sin(p.arm_angles[index]),
                 -math.sin(angle)*math.cos(p.arm_angles[index]),
                 math.cos(angle)]
    thrust_vector = [power*p.max_thrusts[index]*direction[i] for i in range(3)]

    torques = list(np.cross([p.motor_pos[index][i]-p.center[i] for i in range(3)],thrust_vector))
    torques = [torques[i]+power*p.max_torques[index]*direction[i]*p.orientations[index] for i in range(3)]

    return np.array(thrust_vector),np.array(torques)

#Puts all of the motors together
def full_thrust(powers,angles):
    individual = [single_motor(i,powers[i],angles[i]) for i in range(3)]
    force = sum(individual[i][0] for i in range(3))
    torque = sum(individual[i][1] for i in range(3))
    return force,torque