import cv2
import numpy as np
import math
import random
from PIL import Image, ImageOps, ImageDraw
import urllib.request
import time


class Node:
    """Class to store the RRT graph"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.history = []

    def add_to_history(self, node):
        self.history.append(node)


# check collision
def collision(node1, node2, img):
    if abs(node2.x - node1.x) < 1e-6:
        # Vertical line
        y_vals = np.linspace(node1.y, node2.y, 100)
        x_vals = np.full_like(y_vals, node1.x)
    else:
        x_vals = np.linspace(node1.x, node2.x, 100)
        y_vals = ((node2.y - node1.y) / (node2.x - node1.x)) * (x_vals - node1.x) + node1.y

    for i in range(len(x_vals)):
        xi, yi = int(x_vals[i]), int(y_vals[i])
        if 0 <= yi < img.shape[0] and 0 <= xi < img.shape[1]:
            if img[yi, xi] == 0:
                return True  # collision
    return False  # no collision


# check the collision with obstacle and trim
def steer(node1, node2, step_size, goal_node, img):
    x1, y1, x2, y2 = node1.x, node1.y, node2.x, node2.y
    hy, hx = img.shape
    _, theta = dist_and_angle(x1, y1, x2, y2)
    x = x1 + step_size * np.cos(theta)
    y = y1 + step_size * np.sin(theta)
    new_node = Node(x, y)

    if y < 0 or y > hy or x < 0 or x > hx:
        goal_connect = False
        node_connect = False
    else:
        # check direct connection
        if collision(new_node, goal_node, img):
            goal_connect = False
        else:
            goal_connect = True
        # check connection between two nodes
        if collision(node1, new_node, img):
            node_connect = False
        else:
            node_connect = True
    return (new_node, goal_connect, node_connect)


# return dist and angle b/w new point and nearest node
def dist_and_angle(x1, y1, x2, y2):
    dist = math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))
    angle = math.atan2(y2 - y1, x2 - x1)
    return (dist, angle)


# return the nearest node index
def find_nearest_node(node, node_list):
    temp_dist = []
    for i in range(len(node_list)):
        dist, _ = dist_and_angle(node.x, node.y, node_list[i].x, node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))


# generate a random point in the image space
def get_rand_node(h, l):
    rand_y = random.randint(0, h)
    rand_x = random.randint(0, l)
    return Node(rand_x, rand_y)


def RRT(img, img_display, start_node, goal_node, step_size):
    h, l = img.shape  # dim of the loaded image

    # node list
    node_list = []

    # insert the starting point in the node class
    start_node.add_to_history(start_node)
    node_list.append(start_node)

    # display start and goal
    cv2.circle(img_display, (start_node.x, start_node.y), 5, (0, 0, 255), thickness=3, lineType=8)
    cv2.circle(img_display, (goal_node.x, goal_node.y), 5, (0, 0, 255), thickness=3, lineType=8)

    i = 1
    pathFound = False
    while pathFound == False:
        # get rand node
        rand_node = get_rand_node(h, l)

        # find nearest point on the tree (map)
        nearest_node_ind = find_nearest_node(rand_node, node_list)
        nearest_node = node_list[nearest_node_ind]

        # steer
        new_node, goal_connect, node_connect = steer(nearest_node, rand_node,
                                                      step_size, goal_node, img)

        # check the results
        if goal_connect and node_connect:
            new_node.history = nearest_node.history.copy()
            new_node.add_to_history(new_node)
            new_node.add_to_history(goal_node)

            # display
            cv2.circle(img_display, (int(new_node.x), int(new_node.y)), 2, (0, 0, 255), thickness=3, lineType=8)
            cv2.line(img_display, (int(new_node.x), int(new_node.y)), (int(nearest_node.x), int(nearest_node.y)), (0, 255, 0), thickness=1, lineType=8)
            cv2.line(img_display, (int(new_node.x), int(new_node.y)), (int(goal_node.x), int(goal_node.y)), (255, 0, 0), thickness=2, lineType=8)

            # plot the full path
            print("Path has been found")
            pathFound = True
            for j in range(len(new_node.history) - 1):
                cv2.line(img_display, (int(new_node.history[j].x), int(new_node.history[j].y)), (int(new_node.history[j + 1].x), int(new_node.history[j + 1].y)), (255, 0, 0), thickness=2, lineType=8)
            
            # Display the result
            cv2.imshow("RRT Path", img_display)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            break

        elif node_connect:
            new_node.history = nearest_node.history.copy()
            new_node.add_to_history(new_node)
            node_list.append(new_node)

            # display
            cv2.circle(img_display, (int(new_node.x), int(new_node.y)), 2, (0, 0, 255), thickness=3, lineType=8)
            cv2.line(img_display, (int(new_node.x), int(new_node.y)), (int(nearest_node.x), int(nearest_node.y)), (0, 255, 0), thickness=1, lineType=8)
            
            # Show progress (optional - uncomment to see animation)
            # cv2.imshow("RRT Progress", img_display)
            # cv2.waitKey(1)
            continue

        else:
            continue


def draw_circle(event, x, y, flags, param):
    global coordinates
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img2, (x, y), 5, (255, 0, 0), -1)
        coordinates.append(x)
        coordinates.append(y)


if __name__ == "__main__":
    # Image path (raw GitHub URL)
    # you can choose from world1, world2, world3, world4
    imagePath = "https://github.com/asu-iris/course_robotics/raw/main/lec6-mp/figures/world4.png"

    # load image directly from the URL
    resp = urllib.request.urlopen(imagePath)
    image_array = np.asarray(bytearray(resp.read()), dtype=np.uint8)

    img = cv2.imdecode(image_array, cv2.IMREAD_GRAYSCALE)  # grayscale
    img_color = cv2.imdecode(image_array, cv2.IMREAD_COLOR)  # color

    start_node = Node(10, 190)  # starting coordinate
    goal_node = Node(520, 300)  # target coordinate

    step_size = 10  # stepsize for RRT

    # run the RRT algorithm
    RRT(img, img_color, start_node, goal_node, step_size)

