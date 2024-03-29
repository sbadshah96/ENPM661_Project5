import math
import heapdict
import numpy as np
import time
import vidmaker
from sortedcollections import OrderedSet
import pygame
import random

# Define new obstacles based on user input buffer
def obstacles_rec(obstacle_buffer, robot_size):
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

def obstacles_circ(obstacle_buffer, robot_size):
    a = 400
    b = 110
    c = 50 + obstacle_buffer + robot_size

    return a, b, c

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
    elif ((x - a1_circ) ** 2 + (y - b1_circ) ** 2 <= c1_circ ** 2):
        return False
    else:
        return True

def random_point():
    rand_x = random.randint(0, map_x)
    rand_y = random.randint(0, map_y)
    return (rand_x, rand_y)

def find_distance(x1, y1, x2, y2):
    return round(np.sqrt((x1 - x2)**2 + (y1 - y2)**2),2)

def find_closest_node(point):
    min_distance = np.inf
    closest_node = init_pos
    for i in range(len(explored_nodes)):
        x_current = explored_nodes[i][0]
        y_current = explored_nodes[i][1]
        distance = find_distance(point[0],point[1],x_current,y_current)
        if distance < min_distance:
            min_distance = distance
            closest_node = (x_current, y_current)
    return closest_node

def get_angle(node1, node2):
    if node1[0] != node2[0]:
        theta = np.rad2deg(np.arctan(abs(node1[1] - node2[1]) / abs(node1[0] - node2[0])))
        if node1[0] < node2[0] and node1[1] <= node2[1]:
            return np.round(np.deg2rad(theta),2)
        elif node1[0] > node2[0] and node1[1] <= node2[1]:
            return np.round(np.deg2rad(theta+90),2)
        elif node1[0] > node2[0] and node1[1] >= node2[1]:
            return np.round(np.deg2rad(theta+180),2)
        elif node1[0] < node2[0] and node1[1] >= node2[1]:
            return np.round(np.deg2rad(theta+270),2)
    else: 
        if node1[1] >= node2[1]:
            return np.round(np.deg2rad(270),2)
        elif node1[1] < node2[1]:
            return np.round(np.deg2rad(90),2)

def get_new_node(closest_node,theta,new_point):
    for i in range(1, step + 1):
        new_node = ((closest_node[0] + i * np.cos(theta)),(closest_node[1] + i * np.sin(theta)))
        if not check_obstacles(new_node[0], new_node[1]):
            return None
    new_node = (int(new_node[0]),int(new_node[1]))
    if new_node not in explored_nodes:
        parent_node, cost = find_parent(new_node)
        if parent_node != None:
            total_cost = cost + node_records[str(parent_node)][1]
            node_records[str(new_node)] = parent_node,total_cost
            explored_nodes.append(new_node)
            visited_nodes_track.add(new_node)
            rand_points.append(new_point)
            return new_node
        # else:, new
        #     return None
        else:
            None
    else:
        return None

def find_range(node):
    rangeX = np.arange(int(node[0]) - check_radius_RRTS, 
                       int(node[0]) + check_radius_RRTS + 1, 1)
    rangeY = np.arange(int(node[1]) - check_radius_RRTS, 
                       int(node[1]) + check_radius_RRTS + 1, 1)
    return rangeX,rangeY

def find_neighbors(node):
    neighbors = []
    range_x, range_y = find_range(node)
    for i in range_x:
        i = int(i)
        for j in range_y:
            j = int(j)
            coord = (i,j)
            if coord in explored_nodes:
                neighbors.append(coord)
    return neighbors

def find_parent(node):
    min_cost = np.inf
    parent_node = None
    neighbor_nodes = find_neighbors(node)
    for neighbor in neighbor_nodes:
        if neighbor != node:
            # val = check_line_obstacles(node,neighbor)
            # if val == True :
                cost = find_distance(node[0],node[1],neighbor[0],neighbor[1])
                total_cost = cost + node_records[str(neighbor)][1]
                if total_cost < min_cost:
                    min_cost = total_cost
                    parent_node = neighbor
    if parent_node == None:
        return None,None
    else:
        return parent_node,min_cost

def check_goal_reach(x,y,i):
    if find_distance(x, y, goal_pos[0], goal_pos[1]) < goal_radius:
        print('Explored Nodes length:', len(explored_nodes))
        print('Node Records Length: ',len(node_records))
        print('Goal Reached!')
        print('Backtracking path:')
        print(backtracking((x, y)))
        end = time.time()
        print('Time: ', round((end - start), 2), 's')
        print('Iterations',i)
        return True

def check_last_iteration(iter):
    if iter == iterations - 1:
        print('Explored Nodes length:', len(explored_nodes))
        print('Node Records Length: ',len(node_records))
        print('Ran out of fuel.')
        end = time.time()
        print('Time: ', round((end - start), 2), 's')
        viz()

# Finding the optimal path
def backtracking(last_node):
    backtrack.append(last_node)
    key = node_records[str(last_node)][0]
    backtrack.append(key)
    ii = 0
    while key != init_pos:
        key = node_records[str(key)][0]
        # print('key',key)
        backtrack.append(key)
        ii += 1
    return backtrack[::-1]

# Convert coordinates into pygame coordinates
def to_pygame(coords, height):
    return coords[0], height - coords[1]

# Convert an object's coordinates into pygame coordinates
def rec_pygame(coords, height, obj_height):
    return coords[0], height - coords[1] - obj_height

def find_intersection(m1, m2, c1, c2, a, b):
    A = np.array([[-m1, a], [-m2, b]])
    B = np.array([c1, c2])
    X = np.linalg.solve(A, B)
    return X

# Plot arrow
def arrow(screen, lcolor, tricolor, start, end, trirad):
    pygame.draw.line(screen, lcolor, start, end, 1)
    rotation = math.degrees(math.atan2(start[1]-end[1], end[0]-start[0]))+90
    pygame.draw.polygon(screen, tricolor, ((end[0]+trirad*math.sin(math.radians(rotation)), 
                                            end[1]+trirad*math.cos(math.radians(rotation))),
                                           (end[0]+trirad*math.sin(math.radians(rotation-120)),
                                            end[1]+trirad*math.cos(math.radians(rotation-120))),
                                           (end[0]+trirad*math.sin(math.radians(rotation+120)), 
                                            end[1]+trirad*math.cos(math.radians(rotation+120)))))

# Pygame Visualization
def viz():
    pygame.init()
    video = vidmaker.Video("rrt_star_experiment2_"+str(iterations)+"_"+str(check_radius_RRTS)+".mp4", late_export=True)
    size = [600, 200]
    d = obstacle_buffer + robot_size
    monitor = pygame.display.set_mode(size)
    pygame.display.set_caption("RRT* Arena")

    Done = False
    clock = pygame.time.Clock()
    while not Done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                Done = True
        monitor.fill("black")

        # Walls
        pygame.draw.rect(monitor, "red", [0, 0, d, 200], 0)
        pygame.draw.rect(monitor, "red", [0, 0, 600, d], 0)
        pygame.draw.rect(monitor, "red", [0, 200 - d, 600, d], 0)
        pygame.draw.rect(monitor, "red", [600 - d, 0, d, 200], 0)

        # Rectangles
        x, y = rec_pygame([250 - d, 0], 200, 125 + d)
        pygame.draw.rect(monitor, "red", [x, y, 15 + 2 * d, 125 + d], 0)

        x, y = rec_pygame([150 - d, 75 - d], 200, 125 + d)
        pygame.draw.rect(monitor, "red", [x, y, 15 + 2 * d, 125 + d], 0)

        x, y = rec_pygame([250, 0], 200, 125)
        pygame.draw.rect(monitor, "orange", [x, y, 15, 125], 0)

        x, y = rec_pygame([150, 75], 200, 125)
        pygame.draw.rect(monitor, "orange", [x, y, 15, 125], 0)

        # Circle
        pygame.draw.circle(monitor, "red", to_pygame((400, 110), 200), radius=50 + d)
        pygame.draw.circle(monitor, "orange", to_pygame((400, 110), 200), radius=50)

        # Simulation of visited nodes and Backtracking
        for l in range(len(visited_nodes_track) - 2):
            m = visited_nodes_track[l]
            # my_string = "'" + str(m) + "'"
            # print(my_string)
            # print("HELLO")
            n = (node_records[str(m)][0])
            m = to_pygame(m, 200)
            n = to_pygame(n, 200)
            video.update(pygame.surfarray.pixels3d(monitor).swapaxes(0, 1), inverted=False)
            arrow(monitor, "white", (0, 0, 0), [m[0], m[1]], [n[0], n[1]], 0.5)
            pygame.display.flip()
            clock.tick(10000)
        for i in backtrack:
            pygame.draw.circle(monitor, (0, 255, 0), to_pygame(i, 200), 2)
            video.update(pygame.surfarray.pixels3d(monitor).swapaxes(0, 1), inverted=False)
            pygame.display.flip()
            clock.tick(20)

        pygame.display.flip()
        pygame.time.wait(2000)
        Done = True

    pygame.quit()
    # video.export(verbose=True)


robot_size = 10.5

map_x = 600
map_y = 200

init_pos = (int(500), int(100))
goal_pos = (int(100), int(180))
goal_radius = int(5)

iterations = 10000

node_records = {}
explored_nodes = []
visited_nodes_track = OrderedSet()
rand_points = []
backtrack = []

obstacle_buffer = 5
obstacles_var1 = obstacles_rec(obstacle_buffer,robot_size)
obstacles_var2 = obstacles_circ(obstacle_buffer,robot_size)

print('Initial position in obstacle?:', not check_obstacles(init_pos[0],init_pos[1]))
print('Final position in obstacle?:', not check_obstacles(goal_pos[0],goal_pos[1]))

step = 5
check_radius_RRTS = 5

if __name__ == '__main__':
    start = time.time()
    node_records[str(init_pos)] = init_pos,0
    explored_nodes.append(init_pos)
    visited_nodes_track.add(init_pos)
    for i in range(iterations):
        new_point = random_point()
        closest_node = find_closest_node(new_point)
        angle = get_angle(closest_node,new_point)
        new_node = get_new_node(closest_node,angle,new_point)
        if new_node != None:
            val = check_goal_reach(new_node[0], new_node[1],i)
            if val:
                end = time.time()
                print('Time: ', round((end - start), 2), 's')
                viz()
                break
        check_last_iteration(i)
    print('The end')