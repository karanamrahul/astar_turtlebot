# Importing Libraries and Modules
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import queue
import math
import time
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import csv
import tf
import roslib
import sys


wheel_radius = 0.038
wheel_distance = 0.354

"""
@brief This Program will implement the A-star algorithm for a differential robot(Turtle Bot 3)
       with non-holonomic constraints.
       
@author Rahul Karanam

@date 19/04/2022

"""
class Node():
    def __init__(self, state, parent, move, cost, path_array): 

        self.state = state
        self.parent = parent
        self.move = move
        self.cost = cost
        self.pathArray = path_array
        
        
    def getState(self):
        return self.state

    def getPathArray(self):
        return self.pathArray
        
    def getParent(self):
        return self.parent

    def getParentState(self):
        if self.getParent() is None:
            return None
        return self.getParent().getState()
        
    def getMove(self):
        return self.move
        
    def getCost(self):
        return self.cost

    def getFullPath(self):
        
        moves = []
        nodes = []
        current_node = self
        while(current_node.getMove() is not None):

            moves.append(current_node.getMove())
            nodes.append(current_node)
            current_node = current_node.getParent()

        nodes.append(current_node)
        moves.reverse()
        nodes.reverse()
        
        return moves, nodes

    def printStats(self):
        pass
    def __lt__(self,other):
        return self.getState() < other.getState()




class Obstacle:
    
    """ 
    @brief: Class to represent an obstacle in the environment.
    
    Attributes:
    x_min: Minimum x-coordinate of the obstacle.
    x_max: Maximum x-coordinate of the obstacle.
    clearance: Minimum clearance required to move from the obstacle.
    robot_radius: Radius of the robot.
    obstacles points: List of points in the obstacle.
       x: x-coordinate of the obstacle
       y: y-coordinate of the obstacle
        
    Returns: Map with obstacles.
       
    
    """
    def __init__(self, clearance):
        self.x = 10
        self.y = 10
        self.clearance = clearance
        self.robot_radius = 0.354 / 2
        self.clearance = self.robot_radius + self.clearance

        self.circle1_radius = 0.5
        self.circle2_radius = 1
        self.circle1_x_offset = 2
        self.circle1_y_offset = 2
        self.circle2_x_offset = 2
        self.circle2_y_offset = 8

        self.square1_corner1_x = 0.25
        self.square1_corner1_y = 4.25
        self.square_side = 1.5

        self.rect1_corner1_x = 3.75
        self.rect1_corner1_y = 4.25
        self.rect1_length = 2.5
        self.rect1_width = 1.5

        self.rect2_corner1_x = 7.25
        self.rect2_corner1_y = 2
        self.rect2_length = 1.5
        self.rect2_width = 2

    def isInObstacleSpace(self, x, y):

        if (x < 0 or x >= 10 or y < 0 or y >= 10):
          return 1
      
        x_offset = self.circle1_x_offset
        y_offset = self.circle1_y_offset
        radius = self.circle1_radius + self.clearance
        if ((x-x_offset)**2 + (y-y_offset)**2 <= radius**2):
          return 1

        # circle2 obstacle
        x_offset = self.circle2_x_offset
        y_offset = self.circle2_y_offset
        radius = self.circle2_radius + self.clearance
        if ((x-x_offset)**2 + (y-y_offset)**2 <= radius**2):
          return 1

        # square obstacle
        x1 = self.square1_corner1_x - self.clearance
        x2 = x1 + self.square_side + 2*self.clearance
        y1 = self.square1_corner1_y - self.clearance
        y2 = y1 + self.square_side  + 2*self.clearance
        if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
            return 1

        #rectangle obstacle 1
        x1 = self.rect1_corner1_x - self.clearance
        x2 = x1 + self.rect1_length + 2*self.clearance
        y1 = self.rect1_corner1_y - self.clearance
        y2 = y1 + self.rect1_width  + 2*self.clearance
        if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
            return 1

        #rectangle obstacle 2
        x1 = self.rect2_corner1_x - self.clearance
        x2 = x1 + self.rect2_length + 2*self.clearance
        y1 = self.rect2_corner1_y - self.clearance
        y2 = y1 + self.rect2_width  + 2*self.clearance
        if (x >= x1 and x <= x2 and y >= y1 and y <= y2):
            return 1

        return 0
    
    
class Visualization:
    """
    @brief: Class to visualize the environment.
    
    Attributes:
        radius: Radius of the robot.
        clearance: Clearance of the robot.
        
    """
    def __init__(self, obstacle):
        self.radius = obstacle.robot_radius
        self.obstacle = obstacle

    def getRadius(self):
        return self.radius

    def updateMapViz(self, map, state, color):
        X, Y, _ = map.shape
        transformed_y = state[0]
        transformed_x = X - state[1] -1
        map[transformed_x, transformed_y, :] = color

    def addObstacles2Map(self, ax):
        circle1_centre = (self.obstacle.circle1_x_offset, self.obstacle.circle1_y_offset)
        circle1_radius = self.obstacle.circle1_radius +  self.obstacle.clearance

        circle2_centre = (self.obstacle.circle2_x_offset, self.obstacle.circle2_y_offset)
        circle2_radius = self.obstacle.circle2_radius +  self.obstacle.clearance

        square1_corner1 = (self.obstacle.square1_corner1_x - self.obstacle.clearance, self.obstacle.square1_corner1_y - self.obstacle.clearance)
        square1_side = self.obstacle.square_side + (2 * self.obstacle.clearance)

        rect1_corner1 = (self.obstacle.rect1_corner1_x - self.obstacle.clearance, self.obstacle.rect1_corner1_y - self.obstacle.clearance)
        rect1_length = self.obstacle.rect1_length + (2 * self.obstacle.clearance)
        rect1_width = self.obstacle.rect1_width + (2 * self.obstacle.clearance)

        rect2_corner1 = (self.obstacle.rect2_corner1_x - self.obstacle.clearance, self.obstacle.rect2_corner1_y - self.obstacle.clearance)
        rect2_length = self.obstacle.rect2_length + (2 * self.obstacle.clearance)
        rect2_width = self.obstacle.rect2_width + (2 * self.obstacle.clearance)


        circle1 = patches.Circle(circle1_centre, radius=circle1_radius, linewidth=1, edgecolor='orange', facecolor='grey')
        circle2 = patches.Circle(circle2_centre, radius=circle2_radius, linewidth=1, edgecolor='orange', facecolor='grey')
        square1 = patches.Rectangle(square1_corner1, square1_side, square1_side, linewidth=1, edgecolor='orange', facecolor='grey')
        rect1 = patches.Rectangle(rect1_corner1, rect1_length, rect1_width, linewidth=1, edgecolor='orange', facecolor='grey')
        rect2 = patches.Rectangle(rect2_corner1, rect2_length, rect2_width, linewidth=1, edgecolor='orange', facecolor='grey')
        
        ax.set_title('Obstacle Map')
        ax.add_patch(square1)
        ax.add_patch(rect1)
        ax.add_patch(rect2)
        ax.add_patch(circle1)
        ax.add_patch(circle2)
        

        return ax

def chk_goal(current_cell,goal_cell):
    # Calculate the distance between the current cell and the goal cell
    current_cell=current_cell.getState()
    r = np.square(current_cell[0] - goal_cell[0]) + np.square(current_cell[1] - goal_cell[1])
    if r < 1:
        return True
    else :
        return False

def computeHeuristicCost(current_state, goal_state):
    #Calculate the heuristic cost
    cost = 0.0
    if current_state is not None:
        cost =  ((current_state[0]-goal_state[0])**2 + (current_state[1]-goal_state[1])**2)**(0.5)
    return cost

def chk_duplicate(node, node_array, goal_state, threshold=0.5):
    
    """
    @breif: check if the node is already in the node_array
    
    Arguments:  
    node: the node to be checked
    node_array: the array of nodes
    goal_state: the goal state
    threshold: the threshold for the distance between the node and the goal state
    
    Return:
    True: if the node is already in the node_array
    
    """
    result = False
    node_state = node.getState()
    x = node_state[0]
    y = node_state[1]
    theta = node_state[2]
    x = int(half_round(x)/threshold)
    y = int(half_round(y)/threshold)

    if (node.getCost() + computeHeuristicCost(node_state, goal_state) < node_array[x, y, theta]):
        result = True
    return result

def explored_nodes(node, T, rpm1, rpm2, obs):
    """
    @brief: This function checks if the node is in the explored set and if it is, it returns the node.
    
    Args:
        node: The node to be checked.
        T: Time interval
        rpm1: The rpm of the first wheel.
        rpm2: The rpm of the second wheel.
        obs: The obstacle map.

    Returns:
        The node if it is in the explored set, otherwise None.
    
    """
    actions=[[rpm1, rpm1], [rpm2, rpm2], [rpm1, 0], [0, rpm1], [rpm1, rpm2], [rpm2, rpm1], [0, rpm2], [rpm2, 0]]
    state = node.getState()
    branches = []

    for action in actions:
        new_state, path_array, cost = move(state, action, T, obs)
        if new_state is not None:
            branch_node = Node(new_state, node, action, node.getCost() + cost, path_array)
            branches.append(branch_node)
      
    return branches

def move(state, action, T, obs): 
    """
    @breif: Apply the action to the state and return the new state and the cost of the action
    
    Args:
        state: the current state
        action: the action to be applied
        T: the time interval
        obs: the obstacle map
        
    Returns:
        new_state: the new state
        path_array: the path array
        cost: the cost of the action
        
    """
    t = 0
    dt = 0.1
    
    Xi, Yi, thetai = state
    thetai = deg2rad(thetai)

    wL, wR = action

    Xn = Xi
    Yn = Yi
    thetan = thetai

    path_array = []
    cost = 0.0
    path_array.append([Xn, Yn])
    while t<T:
        t = t + dt
        dx = 0.5 * wheel_radius * (wL + wR) * math.cos(thetan) * dt
        dy = 0.5 * wheel_radius * (wL + wR) * math.sin(thetan) * dt
        Xn += dx
        Yn += dy
        thetan += (wheel_radius / wheel_distance) * (wL - wR) * dt
        cost += math.sqrt(math.pow(dx,2) + math.pow(dy,2))
        path_array.append([Xn, Yn])
        
        if obs.isInObstacleSpace(Xn, Yn):
            return None, None, None

    thetan = int(rad2deg(thetan)) # To make sure the angle is integer and range is 0-360
    if (thetan >= 360):
        thetan-=360
    if (thetan <= -360):
        thetan+=360
    return [Xn, Yn, thetan] , path_array, cost

def visualize(viz, traversed_nodes, node_path):
    
    """
    @brief Visualize the path
    
    Args:
        viz: the visualization object
        traversed_nodes: the list of nodes traversed
        node_path: the list of nodes in the path

    Returns:
        A Visualization of the path and explored nodes
    """
    fig, ax = plt.subplots(figsize = (10, 10))
    ax.set(xlim=(0, 10), ylim = (0,10))
    ax.set_facecolor('black')
    ax = viz.addObstacles2Map(ax)
    ax.set_aspect("equal")
    ax.set_title('Path generated by A*')
    for node in traversed_nodes:
        xi, yi, _ = node.getState()
        points = node.getPathArray()
        if points is not None:
            for point in points:
                xn, yn = point
                ax.plot([xi, xn], [yi, yn], color="red", linewidth = 0.5 )
                xi, yi = xn, yn
            plt.pause(0.000001)


    for node in node_path:
        xi, yi, _ = node.getState()
        points = node.getPathArray()
        if points is not None:
            for point in points:
                xn, yn = point
                ax.plot([xi, xn], [yi, yn], color="blue", linewidth = 1.5 )
                xi, yi = xn, yn  
            plt.pause(0.0001)

    plt.show()
    plt.close()





# Round to nearest 0.5
def half_round(n):
    val = round(2*n)/2
    if (val == 10):
      val -= 0.5
    return val

# Convert degrees to radians
def deg2rad(angle):
    return np.pi * angle / 180

# Convert radians to degrees
def rad2deg(angle):
    return 180 * angle / np.pi

# Update the map with the path
def updateMap(map, node, color):
    
    if node.getParent() is not None:
        parent_state = node.getParent().getState()
        current_state = node.getState()
        parent_state_transformed = transformPoint(parent_state, map)
        current_state_transformed = transformPoint(current_state, map)

        map = cv2.line(map, (parent_state_transformed[1], parent_state_transformed[0]), (current_state_transformed[1], current_state_transformed[0]), color, 1)
    else:
        current_state = node.getState()
        current_state_transformed = transformPoint(current_state, map)
        map[current_state_transformed[0], current_state_transformed[1], :] = color
    return map

def updateMapViz(map, state, color):
    X, Y, _ = map.shape
    transformed_y = state[0]
    transformed_x = X - state[1] -1
    map[transformed_x, transformed_y, :] = color 
    return map

def addObstacles2Map(map):

    #circle
    for i in range(circle_offset_x - circle_radius, circle_offset_x + circle_radius):
        for j in range(circle_offset_y - circle_radius, circle_offset_y + circle_radius):
            if (i - circle_offset_x) **2 + (j - circle_offset_y)**2 <= circle_radius**2:
                updateMapViz(map, [i, j], [0, 130, 190])

    for i in range(circle_offset_x - circle_radius, circle_offset_x + circle_radius):
        for j in range(circle_offset_y - circle_radius, circle_offset_y + circle_radius):
            if (i - circle_offset_x) **2 + (j - circle_offset_y)**2 <= (circle_radius - total_clearance)**2:
                updateMapViz(map, [i, j], [0, 130, 190])

    #ellipse
    for i in range(ellipse_offset_x - ellipse_radius_x, ellipse_offset_x + ellipse_radius_x):
        for j in range(ellipse_offset_y - ellipse_radius_y, ellipse_offset_y + ellipse_radius_y):
            if ((i - ellipse_offset_x)/ellipse_radius_x) **2 + ((j - ellipse_offset_y)/ellipse_radius_y)**2 <= 1:
                updateMapViz(map, [i, j], [0, 130, 190])

    for i in range(ellipse_offset_x - ellipse_radius_x, ellipse_offset_x + ellipse_radius_x):
        for j in range(ellipse_offset_y - ellipse_radius_y, ellipse_offset_y + ellipse_radius_y):
            if ((i - ellipse_offset_x)/(ellipse_radius_x - total_clearance)) **2 + ((j - ellipse_offset_y)/(ellipse_radius_y - total_clearance))**2 <= 1:
                updateMapViz(map, [i, j], [0, 130, 190])


    #Obstacle C - Space
    for i in range(c_min_x, c_max_x):
        for j in range(c_min_y, c_max_y):
            if (i <= c_corner4_x):
                updateMapViz(map, [i, j], [0, 130, 190])
            if (j >= c_corner5_y) or (j <= c_corner3_y):
                updateMapViz(map, [i, j], [0, 130, 190])

    for i in range(c_offset_x, c_offset_x + c_length_x):
        for j in range(c_offset_y, c_offset_y + c_length_y):
            if (i <= (c_offset_x + c_width)):
                updateMapViz(map, [i, j], [0, 130, 190])
            if (j >= c_offset_y + c_height + c_width) or (j <= c_offset_y + c_width):
                updateMapViz(map, [i, j], [0, 130, 190])



    # rectangle
    for i in range(rect_x_min, rect_x_max):
        for j in range(rect_y_min, rect_y_max):
                    updateMapViz(map, [i, j],[0, 130, 190])

                   
                    d1 = abs((j - 0.7002*i - 74.39) / (1 + (0.7002)**2)**(0.5))
                    d2 = abs((j - 0.7002*i - 98.8) / (1 + (0.7002)**2)**(0.5))
                    d3 = abs((j + 1.428*i - 176.55) / (1 + (1.428)**2)**(0.5))
                    d4 = abs((j + 1.428*i - 439.44) / (1 + (1.428)**2)**(0.5))
                    if (d1+d2 <= rect_width and d3+d4 <= rect_length):
                        updateMapViz(map, [i, j], [0, 130, 190])

          

    c4_x = int(rect_offset_x - (rect_width - 2 * total_clearance) * np.sin(rect_angle))
    c4_y = int(rect_offset_y + (rect_width - 2 * total_clearance) * np.cos(rect_angle))

    c2_x = int(rect_offset_x + (rect_length - 2 * total_clearance) * np.cos(rect_angle))
    c2_y = int(rect_offset_y + (rect_length - 2 * total_clearance) * np.sin(rect_angle))

    c3_x = int(c2_x - (rect_width - 2 * total_clearance) * np.sin(rect_angle))
    c3_y = int(c2_y + (rect_width - 2 * total_clearance) * np.cos(rect_angle))

    for i in range(rect_x_min, rect_x_max):
        for j in range(rect_y_min, rect_y_max):
            if (j >= (np.tan(rect_angle) * (i - rect_offset_x)  + rect_offset_y)) and (j <= (np.tan(rect_angle) * (i -c4_x)  + c4_y)):
                if (j >= (-np.tan(np.pi/2 - rect_angle) * (i -c4_x)  + c4_y)) and (j <= (-np.tan(np.pi/2 - rect_angle) * (i -c3_x)  + c3_y)):
                    updateMapViz(map, [i, j], [0, 130, 190])
    return map


        
        
wheel_radius = 0.038
wheel_distance = 0.354

def read_file(file_name):

    rows= []
    with open(file_name, 'r') as csvfile:
        # creating a csv reader object
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            rows.append(row)

    return rows


def update_path_msg(path_points):

    path = Path()
    path.header.frame_id = "/map"
    
    for point in path_points:
        x = float(point[0])
        y = float(point[1])

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        
        path.poses.append(pose)

    return path
 
def getLinearAnglularVel(w1, w2):
    v = (wheel_radius / 2) * (w1 + w2)
    w = (wheel_radius / wheel_distance) * (w1 - w2)
    return v, w

def get_twist_msg(move):
    vel = Twist()
    w1 = float(move[0])
    w2 = float(move[1])
    print(w1, w2)

    lin_vel, ang_vel = getLinearAnglularVel(w1, w2)

    vel.linear.x = lin_vel
    vel.linear.y = 0
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = ang_vel

    return vel


def getNearestPoint(current_position, path_points):
    current_x = current_position.position.x
    current_y = current_position.position.y
    current_point = np.array([current_x, current_y]).reshape(-1,2)
    path_points = np.array(path_points, dtype = float).reshape(-1, 2)
    diff = path_points - current_point
    sd = np.sum(np.square(diff), axis = 1)
    idx = np.argmin(sd)
    min_dist = np.sqrt(sd[idx])
    delx, dely = diff[idx, 0], diff[idx, 1]

    if (delx < 0):
        min_dist = min_dist
    else:
        min_dist = -min_dist

    return path_points[idx, 0], path_points[idx, 1], min_dist


def getYaw(rot):
    euler = tf.transformations.euler_from_quaternion(rot)
    yaw = euler[2]
    return yaw

def getCurrectPosition(trans, rot):
    current_pose = Pose()
    current_pose.position.x = trans[0]
    current_pose.position.y = trans[1]
    current_pose.position.z = trans[2]

    current_pose.orientation.x = rot[0]
    current_pose.orientation.y = rot[1]
    current_pose.orientation.z = rot[2]
    current_pose.orientation.w = rot[3]

    return current_pose
    
def updateOmega(vel, d):
    k  = 1.7
    new_vel = vel
    omega = vel.angular.z
    omega_new = omega + k * d
    new_vel.angular.z = omega_new
    return new_vel
    
    
def astar(start_pos, goal_pos, rpm1, rpm2, clearance):
    """
    @brief This function will implement the A-star algorithm for a differential robot
    
    Args:
        start_state: The start state of the robot
        goal_state: The goal state of the robot
        wheel rpm: The wheel rpm of the robot
        clearance: The clearance of the robot
        
    Returns:
       Optimal Path: The optimal path from start to goal
       Direction: The direction of the robot
       Final Points: The final points of the robot
    
    
    """
    
    start_time = time.time()
    # The map is a 2D array of size (m,n)
    height,width = 10,10
    threshold = 0.5
    robot_directions =[]
    robot_pts = []
    robot_path = []
    
    # # Initializing the start and goal states
    # x = input("Enter the start state x: ")
    # y= input("Enter the start state y: ")
    # theta = input("Enter the start state theta: ")
    # start_pos = np.array([x,y,theta]).astype(np.int32)
    # x_g = input("Enter the goal state x: ")
    # y_g = input("Enter the goal state y: ")
    # goal_pos = np.array([x_g,y_g]).astype(np.int32)
    # rpm1= input("Enter the rpm1 : ")
    # rpm2 = input("Enter the rpm2 : ")
    # rpm1,rpm2 = int(rpm1),int(rpm2)
    # clearance = input("Enter the clearance : ")
    # print("\n")
    print("##############################################################################")
    print("                     A-Star  Algorithm is running...                            ")
    print("Start state: ",start_pos)
    print("Goal state:  ",goal_pos)
    print("##############################################################################")
    print("\n")
    
    
    pq = queue.PriorityQueue() # Open list of nodes
    init_node = Node(start_pos, None, None, 0, None)
    pq.put((init_node.getCost(), init_node)) # Inserting the initial node into the priority queue
    traversed_nodes = [] # Closed list of nodes

    obstcale_space = Obstacle(0.0) # Obstacle class object
    map = Visualization(obstcale_space) # Visualization class object

    fig, ax = plt.subplots(figsize = (10, 10)) # Plotting the map
    ax.set(xlim=(0, 10), ylim = (0,10)) 
    ax = map.addObstacles2Map(ax) # Adding the obstacles to the map
    ax.set_aspect("equal")
    ax.set_title("A-Star Algorithm")

    goal_reached = False
    node_array = np.array([[[math.inf for k in range(360)] for j in range(int(height/threshold))] for i in range(int(width/threshold))]) 

    full_path = None # # Full path from start to goal Flag
    goal_reached = False # Flag to check if goal is reached
    



    while (not pq.empty()): # Checking all the explored nodes
        current_node = pq.get()[1] # Popping the node with the lowest cost
        traversed_nodes.append(current_node) # Adding the node to the closed list

        if chk_goal(current_node, goal_pos): # Checking if the goal is reached
            end_time = time.time()
            print("\n")
            print("##############################################################################")
            print("                           A-Star Algorithm is done                            ")
            print('Reached goal state in {} seconds'.format(int(end_time - start_time)))
            print("Total Cost taken the path :  ", int(current_node.getCost()))
            directions, node_path = current_node.getFullPath() # Getting the full path

            robot_directions = directions
            #Uncomment the below line to see the path
            visualize(map, traversed_nodes, node_path) # Visualizing the path

            goal_reached = True         # Flag to check if goal is reached
            
          
            
        

            for node in node_path:
                xi, yi, _ = node.getState()
        
                robot_path.append([xi, yi])

                # We need to convert the state to the map coordinates
                points = node.getPathArray()
                
                
                if points is not None:
                    for point in points:
                        xn, yn = point
                        row = [xn, yn]
                    
                        robot_pts.append(row)
                        xi, yi = xn, yn        
           

        else:
            
            # Getting the children of the current node
            exploration = explored_nodes(current_node, 1, rpm1,rpm2, obstcale_space)
            for node in exploration:
                branch_state = node.getState()
                if chk_duplicate(node, node_array, goal_pos, threshold=0.5): # Checking if the node is already visited
                    # Total cost of the node = cost_to_come + cost_to_go
                    node_array[int(half_round(branch_state[0])/threshold), int(half_round(branch_state[1])/threshold), branch_state[2]] = node.getCost() + computeHeuristicCost(branch_state, goal_pos)
                    pq.put((node.getCost() + computeHeuristicCost(branch_state, goal_pos), node))
        if (goal_reached): 
            break

    return robot_directions, robot_path , robot_pts
                
def turtlebot_pub():

    rospy.sleep(3.)
    path_pub = rospy.Publisher('path', Path, queue_size=10)
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('turtlebot_pub', anonymous=True)

    listener = tf.TransformListener()

    start_point = [int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])]
    goal_state = [int(sys.argv[4]), int(sys.argv[5])]
    clearance = int(sys.argv[6])
    rpm1, rpm2 = int(sys.argv[7]), int(sys.argv[8])

    moves, node_points, path_points = astar(start_point, goal_state, rpm1, rpm2, clearance)
    rospy.sleep(3.)

    path = update_path_msg(path_points)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            current_pose = getCurrectPosition(trans, rot)
            dx, dy, min_dist = getNearestPoint(current_pose, path_points)
  

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No tf available between map and baselink......................")
            rospy.sleep(1.)
            continue

        msg_str = "Publishing vel and path %s" % rospy.get_time()


        if len(moves) == 0:
            move = [0,0]
        else:
            move = moves.pop(0)



        vel = get_twist_msg(move)

        if (abs(min_dist) > 0.1 and len(moves) > 0):
           vel =  updateOmega(vel, min_dist)

        path_pub.publish(path)
        vel_pub.publish(vel)
	
        rate.sleep()

if __name__ == '__main__':
    try:
        turtlebot_pub()
    except rospy.ROSInterruptException:
        pass
