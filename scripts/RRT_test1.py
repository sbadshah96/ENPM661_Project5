#!/usr/bin/env python

import math
import heapdict
import numpy as np
import time
import vidmaker
from sortedcollections import OrderedSet
import pygame
import random

# Define new obstacles based on user input buffer
def obstacles_rec(obstacle_buffer, robot_size=10.5):
    obstacles = []

    buffer_val = obstacle_buffer + robot_size

    c1_rec1 = 250
    m1_rec1 = 0
    c1_rec1_new = c1_rec1 - buffer_val
    obstacles.append((m1_rec1, c1_rec1_new))

    c2_rec1 = 265
    m2_rec1 = 0
    c2_rec1_new = c2_rec1 + buffer_val
    obstacles.append((m2_rec1, c2_rec1_new))

    c3_rec1 = 125
    m3_rec1 = 0
    c3_rec1_new = c3_rec1 + buffer_val
    obstacles.append((m3_rec1, c3_rec1_new))

    c1_rec2 = 150
    m1_rec2 = 0
    c1_rec2_new = c1_rec2 - buffer_val
    obstacles.append((m1_rec2, c1_rec2_new))

    c2_rec2 = 165
    m2_rec2 = 0
    c2_rec2_new = c2_rec2 + buffer_val
    obstacles.append((m2_rec2, c2_rec2_new))

    c3_rec2 = 75
    m3_rec2 = 0
    c3_rec2_new = c3_rec2 - buffer_val
    obstacles.append((m3_rec2, c3_rec2_new))


    c1_bound = 0 + buffer_val
    c2_bound = 600 - buffer_val
    c3_bound = 0 + buffer_val
    c4_bound = 200 - buffer_val
    obstacles.append((0, c1_bound))
    obstacles.append((0, c2_bound))
    obstacles.append((0, c3_bound))
    obstacles.append((0, c4_bound))

    return obstacles

def obstacles_circ(obstacle_buffer, robot_size=10.5):
    a = 400
    b = 110
    c = 50 + obstacle_buffer + robot_size

    return a,b,c

def random_point():
    # random_x = np.random.choice(len(map_x),4,replace=False)
    rand_x = random.randint(0,map_x)
    rand_y = random.randint(0,map_y)

    return (rand_x,rand_y)

# Check if the robot is in obstacle space.
def check_obstacles(x, y):
    c1_rec1 = obstacles_var1[0][1]
    c2_rec1 = obstacles_var1[1][1]
    c3_rec1 = obstacles_var1[2][1]

    c1_rec2 = obstacles_var1[3][1]
    c2_rec2 = obstacles_var1[4][1]
    c3_rec2 = obstacles_var1[5][1]

    a1_circ = obstacles_var2[0]
    b1_circ = obstacles_var2[1]
    c1_circ = obstacles_var2[2]

    c1_bound = obstacles_var1[6][1]
    c2_bound = obstacles_var1[7][1]
    c3_bound = obstacles_var1[8][1]
    c4_bound = obstacles_var1[9][1]

    if (((c1_rec1) <= x <= (c2_rec1)) and (0 <= y <= (c3_rec1))):
        return False
    elif (((c1_rec2) <= x <= (c2_rec2)) and ((c3_rec2) <= y <= 200)):
        return False
    elif ((x <= c1_bound) or (x >= c2_bound) or (y <= c3_bound) or (y >= c4_bound)):
        return False
    elif ((x-a1_circ)**2 + (y-b1_circ)**2 <= c1_circ**2):
        return False
    else:
        return True

def find_distance(x1,y1,x2,y2):
    return np.sqrt((x1-x2)**2 + (y1-y2)**2)

def find_closest_node(node):
    min_distance = np.inf
    closest_point = (0,0)
    for i in range(len(explored_nodes)):
        x_current = explored_nodes[i][0]
        y_current = explored_nodes[i][1]
        distance = find_distance(node[0],x_current,node[1],y_current)
        if distance < min_distance:
            min_distance = distance
            closest_point = (x_current,y_current)
    return closest_point

def get_angle(node1,node2):
    if node1[0] != node2[0]:
        theta = np.arctan((node1[1]-node2[1])/(node1[0]-node2[0]))
        return theta
    else:
        return np.deg2rad(90)
    
def get_new_node(node,theta):
    for i in range(1,step+1):
        new_node = int(node[0] + i*np.cos(theta)), int(node[1] + i*np.sin(theta))
        if not check_obstacles(new_node[0],new_node[1]):
            # print('I\'m here.')
            return None
    if new_node not in explored_nodes:
        node_records[str(new_node)] = closest_node
        explored_nodes.append(new_node)
        rand_points.append(new_point)
        return new_node
        # else:
        #     return None
    else:
        return None
        
def check_goal_reach(x,y):
    if find_distance(x,y,goal_pos[0],goal_pos[1]) < goal_radius:
        print('Explored Nodes:')
        print(explored_nodes)
        print('Explored Nodes length:',len(explored_nodes))
        print('Goal Reached!')
        return True

def check_last_iteration(iter):
    if iter==iterations-1:
        print('Explored Nodes:')
        print(explored_nodes)
        print('Explored Nodes length:',len(explored_nodes))
        print('Ran out of fuel.')

init_pos = (50,100)
goal_pos = (55,105)
goal_radius = 5

map_x = 600
map_y = 200

iterations = 6000

node_records = {}
explored_nodes = []
visited_nodes_track = OrderedSet()
rand_points = []

obstacle_buffer = 5
obstacles_var1 = obstacles_rec(obstacle_buffer)
obstacles_var2 = obstacles_circ(obstacle_buffer)

step = 5

if __name__ == '__main__':
    node_records[str(init_pos)] = init_pos
    explored_nodes.append(init_pos)
    for i in range(iterations):
        new_point = random_point()
        closest_node = find_closest_node(new_point)
        angle = get_angle(new_point,closest_node)
        new_node = get_new_node(closest_node,angle)
        if new_node != None:
            val = check_goal_reach(new_node[0],new_node[1])
            if val:
                break
        check_last_iteration(i)
    print('The end')