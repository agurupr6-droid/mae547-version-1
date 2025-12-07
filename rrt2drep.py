import cv2
import numpy as np
import math
import random
import urllib.request
import time
import matplotlib.pyplot as plt


class Node:
    """Class to store the RRT graph"""
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.history = []
    def add_to_history(self,node):
        self.history.append(node)

# check collision
def collision(node1, node2):
    """Check collision along the line segment between two nodes."""
    color = []
    dx = node2.x - node1.x
    dy = node2.y - node1.y
    if abs(dx) < 1e-6:
        # vertical line
        y_vals = np.linspace(node1.y, node2.y, 100)
        x_vals = np.full_like(y_vals, node1.x)
    else:
        x_vals = np.linspace(node1.x, node2.x, 100)
        y_vals = ((dy / dx) * (x_vals - node1.x)) + node1.y
    for xi, yi in zip(x_vals, y_vals):
        xi_int, yi_int = int(xi), int(yi)
        if 0 <= yi_int < img.shape[0] and 0 <= xi_int < img.shape[1]:
            color.append(img[yi_int, xi_int])
    return 0 in color  # True if collision

# check the  collision with obstacle and trim
def steer(node1, node2, step_size, goal_node, img):
    x1, y1, x2, y2 = node1.x, node1.y, node2.x, node2.y
    hy,hx=img.shape
    _,theta = dist_and_angle(x1,y1,x2,y2)
    x=x1 + step_size*np.cos(theta)
    y=y1 + step_size*np.sin(theta)
    new_node=Node(x,y)

    if y<0 or y>hy or x<0 or x>hx:
        print("Point out of image bound")
        goal_connect = False
        node_connect = False
    else:
        # check direct connection
        if collision(new_node,goal_node):
            goal_connect = False
        else:
            goal_connect=True
        # check connection between two nodes
        if collision(node1,new_node):
            node_connect = False
        else:
            node_connect = True
    return(new_node,goal_connect,node_connect)

# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return(dist,angle)

# return the neaerst node index
def find_nearest_node(node, node_list):
    temp_dist=[]
    for i in range(len(node_list)):
        dist,_ = dist_and_angle(node.x,node.y,node_list[i].x,node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

# generate a random point in the image space
def get_rand_node(h,l):
    rand_y = random.randint(0, h)
    rand_x = random.randint(0, l)
    return Node(rand_x,rand_y)


def RRT(img, start_node, goal_node, step_size):
    h,l= img.shape # dim of the loaded image

    # node list
    node_list=[]

    # insert the starting point in the node class
    start_node.add_to_history(start_node)
    node_list.append(start_node)

    i=1
    pathFound = False
    path_points = None
    while pathFound==False:
        # get rand node
        rand_node = get_rand_node(h,l)

        # find nearest point on the tree (map)
        nearest_node_ind = find_nearest_node(rand_node, node_list)
        nearest_node = node_list[nearest_node_ind]

        # steer
        new_node,goal_connect,node_connect = steer(nearest_node,rand_node,
                                                 step_size,goal_node,img)

        # check the results
        if goal_connect and node_connect:
            new_node.history=nearest_node.history.copy()
            new_node.add_to_history(new_node)
            new_node.add_to_history(goal_node)

            print("Path has been found")
            pathFound = True
            path_points = [(n.x, n.y) for n in new_node.history]
            break

        elif node_connect:
            new_node.history=nearest_node.history.copy()
            new_node.add_to_history(new_node)
            node_list.append(new_node)
            continue

        else:
            continue

    return path_points, node_list

def draw_circle(event,x,y,flags,param):
    global coordinates
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img2,(x,y),5,(255,0,0),-1)
        coordinates.append(x)
        coordinates.append(y)


def plot_result(img_gray, path_points, start_node, goal_node):
    """Plot obstacles and path similar to the provided example."""
    h, w = img_gray.shape
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.imshow(255 - img_gray, cmap="gray", origin="upper")
    if path_points:
        xs = [p[0] for p in path_points]
        ys = [p[1] for p in path_points]
        ax.plot(xs, ys, color="red", linewidth=2)
    ax.plot(start_node.x, start_node.y, "bs", markersize=6)
    ax.plot(goal_node.x, goal_node.y, "b+", markersize=10, markeredgewidth=2)
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    plt.tight_layout()
    plt.show()


def generate_simple_world():
    """Generate a simple binary world with two obstacles."""
    h, w = 320, 640  # height, width
    img = np.ones((h, w), dtype=np.uint8) * 255  # free space = 255, obstacle = 0

    # Obstacle 1: circle
    cv2.circle(img, (150, 200), 60, 0, -1)
    # Obstacle 2: rectangle
    box_top_left = (360, 140)
    box_bottom_right = (520, 300)
    cv2.rectangle(img, box_top_left, box_bottom_right, 0, -1)

    start = Node(20, 180)
    # goal at box center
    goal_x = (box_top_left[0] + box_bottom_right[0]) // 2
    goal_y = (box_top_left[1] + box_bottom_right[1]) // 2
    goal = Node(goal_x, goal_y)
    step = 12
    return img, start, goal, step


if __name__ == "__main__":
    img, start_node, goal_node, step_size = generate_simple_world()
    path_points, node_list = RRT(img, start_node, goal_node, step_size)
    plot_result(img, path_points, start_node, goal_node)
