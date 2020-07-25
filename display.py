import pygame
import parameters as p
import numpy as np
import math

#I could comment this but i really dont want to

#td = top-down view
#gy = gyroscope view
arm_width = [6,2]
center_size = [15,5]
motor_size = [9,3]
width = 800
height = 450
zooms = [40,12]
sizes = [center_size,motor_size,motor_size,motor_size]
colors = [(60,60,200),(150,150,60),(60,150,150),(150,60,150)]
framerate = 20
td_start = [width*6/8,height*1/2]
gy_start = [80,height*1/2]
dist_ratio = [0.05,1]

pygame.init()
screen = pygame.display.set_mode((width, height))
screen.fill((255,255,255))
pygame.display.flip()
pygame.event.pump()
clock = pygame.time.Clock()
gy_axes = np.array(((0,1,0),(0,0,-1)))
gy_order_axis = -np.cross(gy_axes[0],gy_axes[1])
gy_height = np.array(((0,0,0),(0,0,0),(0,0,dist_ratio[0])))
td_axes = np.array(((0,1,0),(1,0,0)))
td_order_axis = np.cross(td_axes[0],td_axes[1])

def real_to_frame(pt,axes,base,zoom):
    out = axes.dot(pt)
    return [round(base[0]+zoom*out[0]), round(base[1]+zoom*out[1])]

def get_arm_len(center,motor,axes,zoom):
    out = axes.dot(center-motor)
    return zoom*np.linalg.norm(out)

def get_arm_angle(center,motor,axes):
    out = axes.dot(center-motor)
    return 180/math.pi*math.atan2(out[0],out[1])

def find_order(elements,axis):
    distance = [(axis.dot(elements[i]),i) for i in range(len(elements))]
    distance.sort()
    return (d[1] for d in distance)

#This is awful awful code practice but i was too lazy to not :/
#And too lazy to comment it properly rip
def disp_frame(state):
    gy_arms = [0,0,0]
    gy_arm_rects = [0,0,0]
    screen.fill((255,255,255))
    # gy_body_coords = [state[2].dot(np.array(p.body_pts[k])) + gy_height.dot(state[0]) for k in range(len(p.body_pts))]
    gy_body_coords = [state[2].rotate(np.array(p.body_pts[k])) + gy_height.dot(state[0]) for k in range(len(p.body_pts))]
    for i in range(1,1+len(p.motor_pos)):
        arm = pygame.Surface((arm_width[0], get_arm_len(gy_body_coords[0],gy_body_coords[i],gy_axes,zooms[0])))
        arm.set_colorkey((0, 0, 0))
        arm.fill((60,60,60))
        gy_arms[i-1] = pygame.transform.rotate(arm, get_arm_angle(gy_body_coords[0],gy_body_coords[i],(gy_axes)))
        gy_arm_rects[i-1] = gy_arms[i-1].get_rect()
        gy_arm_rects[i-1].center = (real_to_frame(1/2*(gy_body_coords[0]+gy_body_coords[i]),gy_axes,gy_start,zooms[0]))
    elements = gy_body_coords
    ordering = find_order(elements,gy_order_axis)

    for i in ordering:
        if i != 0:
            screen.blit(gy_arms[i-4],gy_arm_rects[i-4])
        pygame.draw.circle(screen,colors[i],real_to_frame(gy_body_coords[i],gy_axes,gy_start,zooms[0]),sizes[i][0],sizes[i][0])

    td_arms = [0,0,0]
    td_arm_rects = [0,0,0]
    td_body_coords = [gy_body_coords[i] + state[0]*dist_ratio[1] for i in range(len(gy_body_coords))]
    for i in range(1,1+len(p.motor_pos)):
        arm = pygame.Surface((arm_width[1], get_arm_len(td_body_coords[0],td_body_coords[i],td_axes,zooms[1])))
        arm.set_colorkey((0, 0, 0))
        arm.fill((60,60,60))
        td_arms[i-1] = pygame.transform.rotate(arm, get_arm_angle(td_body_coords[0],td_body_coords[i],(td_axes)))
        td_arm_rects[i-1] = td_arms[i-1].get_rect()
        td_arm_rects[i-1].center = (real_to_frame(1/2*(td_body_coords[0]+td_body_coords[i]),td_axes,td_start,zooms[1]))
    elements = td_body_coords
    ordering = find_order(elements,gy_order_axis)

    for i in ordering:
        if i != 0:
            screen.blit(td_arms[i-4],td_arm_rects[i-4])
        pygame.draw.circle(screen,colors[i],real_to_frame(td_body_coords[i],td_axes,td_start,zooms[1]),sizes[i][1],sizes[i][1])

    pygame.display.flip()
    pygame.event.pump()
    clock.tick(framerate)

# print(get_arm_len((0,0),(0.8,0)))