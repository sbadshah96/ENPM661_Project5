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

    return a, b, c


def random_point():
    # random_x = np.random.choice(len(map_x),4,replace=False)
    rand_x = random.randint(0, map_x)
    rand_y = random.randint(0, map_y)

    return (rand_x, rand_y)


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


def find_distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def find_closest_node(node):
    min_distance = np.inf
    closest_point = (0, 0)
    for i in range(len(explored_nodes)):
        x_current = explored_nodes[i][0]
        y_current = explored_nodes[i][1]
        distance = find_distance(node[0],node[1],x_current,y_current)
        if distance < min_distance:
            min_distance = distance
            closest_point = (x_current, y_current)
    return closest_point


def get_angle(node1, node2):
    if node1[0] != node2[0]:
        theta = np.arctan((node1[1] - node2[1]) / (node1[0] - node2[0]))
        return theta
    else:
        return np.deg2rad(90)


# Custom rounding off function for coordinates
def custom_coord_round(a):
    if a - int(a) <= 0.25:
        return int(a)
    elif 0.25 < a - int(a) <= 0.75:
        return int(a) + 0.5
    elif 0.75 < a - int(a) < 1:
        return int(a) + 1


def get_new_node(node, theta):
    for i in range(1, step + 1):
        new_node = (custom_coord_round(node[0] + i * np.cos(theta)),
                    custom_coord_round(node[1] + i * np.sin(theta)))
        if not check_obstacles(new_node[0], new_node[1]):
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


def check_goal_reach(x, y):
    if find_distance(x, y, goal_pos[0], goal_pos[1]) < goal_radius:
        # print('Explored Nodes:')
        # print(explored_nodes)
        # print('Node Records:')
        # print(node_records)
        print('Explored Nodes length:', len(explored_nodes))
        print('Goal Reached!')
        print('Backtracking path:')
        print(backtracking((x, y)))

        return True


def check_last_iteration(iter):
    if iter == iterations - 1:
        # print('Explored Nodes:')
        # print(explored_nodes)
        # print('Node Records:')
        # print(node_records)
        print('Explored Nodes length:', len(explored_nodes))
        print('Ran out of fuel.')


# Finding the optimal path
def backtracking(last_node):
    backtrack.append(last_node)
    key = node_records[str(last_node)]
    backtrack.append(key)
    while key != init_pos:
        key = node_records[str(key)]
        backtrack.append(key)
    return backtrack[::-1]

""" Convert coordinates into pygame coordinates """
def to_pygame(coords, height):
    return coords[0], height - coords[1]


""" Convert an object's coordinates into pygame coordinates """
def rec_pygame(coords, height, obj_height):
    return coords[0], height - coords[1] - obj_height

def find_intersection(m1, m2, c1, c2, a, b):
    A = np.array([[-m1, a], [-m2, b]])
    B = np.array([c1, c2])
    X = np.linalg.solve(A, B)
    return X

init_pos = custom_coord_round(50), custom_coord_round(100)
goal_pos = custom_coord_round(100), custom_coord_round(115)
goal_radius = int(5)

map_x = 600
map_y = 200

iterations = 60000

node_records = {}
explored_nodes = []
visited_nodes_track = OrderedSet()
rand_points = []
backtrack = []

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
        angle = get_angle(new_point, closest_node)
        new_node = get_new_node(closest_node, angle)
        if new_node != None:
            val = check_goal_reach(new_node[0], new_node[1])
            if val:
                break
        check_last_iteration(i)
    print('The end')

    """ Pygame Visualization """
    pygame.init()
    # video = vidmaker.Video("a_star_shreejay_aaqib.mp4", late_export=True)
    size = [600, 200]
    d = obstacle_buffer + 10.5
    monitor = pygame.display.set_mode(size)
    pygame.display.set_caption("Arena")

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
        for l in range(len(explored_nodes) - 2):
            m = explored_nodes[l]
            n = node_records[str(m)]
            m = to_pygame(m, 250)
            n = to_pygame(n, 250)
            print(n)
            # video.update(pygame.surfarray.pixels3d(monitor).swapaxes(0, 1), inverted=False)
            pygame.draw.lines(monitor, "white", False,n, width=1)
            pygame.display.flip()
            clock.tick(500)
        for i in backtrack:
            pygame.draw.circle(monitor, (0, 255, 0), to_pygame(i, 200), 2)
            # video.update(pygame.surfarray.pixels3d(monitor).swapaxes(0, 1), inverted=False)
            pygame.display.flip()
            clock.tick(20)

        pygame.display.flip()
        pygame.time.wait(1000)
        Done = True

    pygame.quit()
    # video.export(verbose=True)