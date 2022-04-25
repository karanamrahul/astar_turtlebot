"""
@brief This Program will implement the A-star algorithm for a differential robot
       with non-holonomic constraints.
       
@author Rahul Karanam

@date 19/04/2022

"""


# Importing Libraries and Modules
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import queue
import math
import csv
import time
from helper import *
from Node import *



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
            
            # Writing the path,nodes and rpm values to a csv file for later implementation on the robot
            
            path_file = open('astar_turtlebot/src/path.csv', 'w')
            path_nodes = open('astar_turtlebot/src/nodes.csv', 'w')
            file_rpm = open('astar_turtlebot/src/rpm.csv', 'w')
            
            path_write = csv.writer(path_file)
            nodes_write = csv.writer(path_nodes)
            rpm_write = csv.writer(file_rpm)
            
            
            # Writing the path to the csv file
            for direction in directions:
                rpm_write.writerow(direction)

            for node in node_path:
                xi, yi, _ = node.getState()
                nodes_write.writerow([xi, yi])
                robot_path.append([xi, yi])

                # We need to convert the state to the map coordinates
                points = node.getPathArray()
                
                
                if points is not None:
                    for point in points:
                        xn, yn = point
                        row = [xn, yn]
                        path_write.writerow(row)
                        robot_pts.append(row)
                        xi, yi = xn, yn        
            path_file.close()
            file_rpm.close()
            path_nodes.close()
        

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
                
