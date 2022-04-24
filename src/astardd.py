"""
@brief This Program will implement the A-star algorithm for a differential robot
       with non-holonomic constraints.
       
@author Rahul Karanam

@date 19/04/2022

"""


# Importing Libraries 
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import queue
import math
import csv
from helper import *
from Node import *



def astar():

    h,w = 10,10
    threshold = 0.5
    start_point = [5,3,0]
    goal_state = [9,9]
    w1, w2 = 5, 10
    nodes = queue.PriorityQueue()
    init_node = Node(start_point, None, None, 0, None)
    nodes.put((init_node.getCost(), init_node))
    traversed_nodes = []

    obs = Obstacle(0.0)
    viz = Visualization(obs)

    fig, ax = plt.subplots(figsize = (10, 10))
    ax.set(xlim=(0, 10), ylim = (0,10))
    ax = viz.addObstacles2Map(ax)
    ax.set_aspect("equal")

    goal_reached = False
    node_array = np.array([[[math.inf for k in range(360)] for j in range(int(h/threshold))] for i in range(int(w/threshold))])

    full_path = None
    goal_reached = False
    print('Finding path.........')



    while (not nodes.empty()):
        current_node = nodes.get()[1]
        traversed_nodes.append(current_node)

        if checkGoalReached(current_node, goal_state,1):
            print('Goal reached')
            print("The cost of path: ", current_node.getCost())
            moves, node_path = current_node.getFullPath()

            visualize(viz, traversed_nodes, node_path)

            goal_reached = True

            fp = open('path_points.csv', 'w')
            fn = open('path_nodes.csv', 'w')
            fv = open('vel_points.csv', 'w')
            writer_p = csv.writer(fp)
            writer_n = csv.writer(fn)
            writer_v = csv.writer(fv)

            for move in moves:
                writer_v.writerow(move)

            for node in node_path:
                xi, yi, _ = node.getState()
                writer_n.writerow([xi, yi])

                points = node.getPathArray()
                if points is not None:
                    for point in points:
                        xn, yn = point
                        row = [xn, yn]
                        writer_p.writerow(row)
                        xi, yi = xn, yn        
            fp.close()
            fv.close()
            fn.close()
        

        else:
            branches = getBranches(current_node, 1, w1, w2, obs)
            for branch_node in branches:
                branch_state = branch_node.getState()
                if checkVisited(branch_node, node_array, goal_state, threshold=0.5):
                    node_array[int(half_round(branch_state[0])/threshold), int(half_round(branch_state[1])/threshold), branch_state[2]] = branch_node.getCost() + computeHeuristicCost(branch_state, goal_state)
                    nodes.put((branch_node.getCost() + computeHeuristicCost(branch_state, goal_state), branch_node))
        if (goal_reached): break

if __name__ == '__main__':
    astar()
