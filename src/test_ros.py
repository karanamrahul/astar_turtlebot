import numpy as np
import cv2
import matplotlib.pyplot as plt
from Priority_queue import PriorityQueue
import math
import pygame
import time


output= cv2.VideoWriter('result.mp4', cv2.VideoWriter_fourcc(*'mp4v'),10, (400, 250))

# We can also use the below function to generate our map
def generate_map():
    """Generating Map using the obstacle space

    Returns:
        [np.narray]: [It is the updated map with obstacle space]
    """
    map = np.zeros((250, 400, 3), np.uint8)

    obs_list = []
    for x in range(0, map.shape[1]):
     for y in range(0, map.shape[0]):
        if is_polygon(x,  y) or is_hexagon(x,  y) or is_circle(x, y):
            map[y][x] = [0, 130, 190]
            obs_list.append((x, y))

    return map, obs_list

def is_polygon(x, y):
    l1 = (0.316 * x + 173.608 - y) >= 0
    l2 = (0.857 * x + 111.429 - y) <= 0
    lm = (-0.114 * x + 189.091 - y) <= 0
    l3 = (-3.2 * x + 436 - y) >= 0
    l4 = (-1.232 * x + 229.348 - y) <= 0

    if (l1 and l2 and lm) or (l3 and l4 and not lm):
         return True
    else:
        return False



def is_hexagon(x, y):
    l1 = (-0.571 * x + 174.286 - y) <= 0
    l2 = (165 - x) <= 0
    l3 = (0.571 * x + 25.714 - y) >= 0
    l4 = (-0.571 * x + 254.286 - y) >= 0
    l5 = (235 - x) >= 0
    l6 = (0.571 * x - 54.286 - y) <= 0

    if l1 and l2 and l3 and l4 and l5 and l6:
        return True
    else:
        return False




def is_circle(x ,y):
    circ_eq = ((x - 300)**2 + (y - 185)**2 - 40*40) <= 0
    if circ_eq:
        return True
    else:
        return False

def polygon_space(x, y):
    l1 = (0.316 * x + 178.608 - y) >= 0
    l2 = (0.857 * x + 106.429 - y) <= 0
    lmid = (-0.114 * x + 189.091 - y) <= 0
    l3 = (-3.2 * x + 450 - y) >= 0
    l4 = (-1.232 * x + 220.348 - y) <= 0

    return ((l1 and l2 and lmid) or (l3 and l4 and not lmid)) 


def hexagon_space(x, y):
    l1 = (-0.575 * x + 169 - y) <= 0
    l2 = (160 - x) <= 0
    l3 = (0.575 * x + 31 - y) >= 0
    l4 = (-0.575 * x + 261 - y) >= 0
    l5 = (240 - x) >= 0
    l6 = (0.575 * x - 61 - y) <= 0

    return l1 and l2 and l3 and l4 and l5 and l6

def circle_space(x, y):
    circ = ((x - 300) ** 2 + (y - 185) ** 2 - 45 * 45) <= 0
    
    return circ

def is_obstacle(y,x):
    
    if  is_circle(x, y) or is_hexagon(x,y) or is_polygon(x,y):
        return True
    else: return False


def check_obs_space(y_pos, x_pos):
   
    return polygon_space(x_pos, y_pos) or hexagon_space(x_pos, y_pos) or circle_space(x_pos, y_pos) \
        or (maze.shape[1] - 5 <= x_pos) or x_pos <= 4 or (maze.shape[0] - 5 <= y_pos) or y_pos <= 4


#To check the origin
#cv2.circle(map, [0, 0], 10, (0, 0, 255), -1)


def heuristicEuclidean(now, goal):
    now = (now[0], now[1])
    cost=np.sqrt((now[0]-goal[0])**2 + (now[1]-goal[1])**2)
    return cost

# This function is used to check whether a position is a legal position or not 
# considering the obstacle space and checking the tolerance of the robot with respect 
# to the obstacle space
def is_legal_pos(map, pos):
    i,j= pos
    num_rows = len(map)
    num_cols = len(map[0])
    return 0 <= i < num_rows and 0 <= j < num_cols and not is_obstacle(i,j) and not check_obs_space(i,j)


"""
This function is used to generate the path a.k.a back-tracing
It returns two variables 
[path] - Shortest path found using the algorithm
[dict_path] - This dictionary contains all the explored nodes in the map used.
"""
def get_path(dict_path, start, goal,turtleaction,linalg):
    path = []
    pos = goal
    while pos != start:
        path.append(pos)
        pos = dict_path[pos]
    path.append(start)
    path.reverse()
    return path,dict_path,turtleaction,linalg

# This represent all the actions needed in a 8-action space  
def action_space_rpm(wheel_rpm):
   rpm1,rpm2=wheel_rpm
   actions_rpm = {"(0,rpm1)": (0,rpm1), "(rpm1,0)": (rpm1,0), "(rpm1,rpm1)": (rpm1,rpm1), "(0,rpm2)": (0,rpm2), "(rpm2,0)": (rpm2,0), "(rpm2,rpm2)": (rpm2,rpm2), "(rpm1,rpm2)": (rpm1,rpm2), "(rpm2,rpm1)": (rpm2,rpm1)}
   return actions_rpm




def pos2coords(pos,step_size=1):
    x,y,theta=pos
    x_new = int(round((step_size*np.cos(np.deg2rad(theta)) + x),2))
    y_new = int(round((step_size*np.sin(np.deg2rad(theta)) + y),2))
    return (x_new,y_new)

def cost_rpm(current_cell,Thetai,UL,UR):
    X,Y= current_cell
    t = 0
    r = 0.38
    L = 3.54
    dt = 0.1
    Xs=X
    Ys=Y
    T = Thetai
    cost=0
    
    
    while t<1:
        t = t + dt
        Xs = X
        Ys = Y
        dx = 0.5*r * (UL + UR) * math.cos(T*math.pi/180) * dt
        dy = 0.5*r * (UL + UR) * math.sin(T*math.pi/180) * dt
        dw = (r / L) * (UR - UL) * dt
        dw = dw * (180/math.pi)
        T+=dw
        cost=cost+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(T*math.pi/180) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(T*math.pi/180) * dt),2))
        X+=dx
        Y+=dy        
    if T>= 360 or T < 0:
                T = T % 360
    
    return X,Y,T,cost,dx,dy,dw
    


def chk_duplicate(V,pos,t):
    return V[2*int(np.round(pos[0]))][2*int(np.round(pos[1]))][int(round_theta(t))] != 1

def move_turtle(publisher,dx,dy,dw):
    rate = rospy.Rate(100)
    twist = Twist()
    vel = np.sqrt(dx**2 + dy**2)/0.1
    end_time = rospy.get_time() + 0.1
    while rospy.get_time() < end_time:
        twist.linear.x = vel
        twist.angular.z = dw
        publisher.publish(twist)
        rate.sleep()
        
def plot_curve(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    dw = 3.14 * Thetai / 180


# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, dw: End point coordintes
    D=0
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(dw) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(dw) * dt
        dw += (r / L) * (UR - UL) * dt
        plt.plot([Xs, Xn], [Ys, Yn], color="blue")
        
    dw = 180 * (dw) / 3.14
    return Xn, Yn, dw, D

def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1-x2) + abs(y1-y2)

def theta2ind(theta):
    return int(round(theta/15))
    
def round_theta(theta):
    T = int((round(theta/15)*15)//15)
    if T == 24:
        T = 0
    return T
    
def chkgoal(current,goal):
    
    if np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2) <= 10:
        return True
    else:
        return False
def display(predec,result,obs_list):
    Rat =0.5
    # Put animation here

    #creating an empty canvas
    canvas = np.zeros((int(250/Rat),int(400/Rat),3),np.uint8)

    # list of all obstacles to put in canvas
    for c in obs_list: #change the name of the variable l
        x = c[1]
        y = c[0]
        canvas[(x,y)]=[0,255,255] #assigning a yellow coloured pixel
    canvas = np.flipud(canvas)
    canvas_for_backtrack = canvas.copy()
    canvas_for_visited = canvas.copy()
    canvas_for_visited = cv2.resize(canvas_for_visited,(1200,750))
    # #showing the obstacle map
    cv2.imshow('canvas',canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    pygame.init()

    display_width = int(400/Rat)
    display_height = int(250/Rat)
    n = 2
    m = n
    # gameDisplay = pygame.display.set_mode((display_width,display_height),0, 32, 0, 10)
    gameDisplay = pygame.display.set_mode((display_width,n*display_height))
    # gameDisplay = pygame.display.set_mode((1920,1080),flags, vsync =1)
    # gameDisplay = pygame.display.set_mode((0,0),pygame.FULLSCREEN)
    pygame.display.set_caption('Visited Nodes- Animation')
    black = (0,0,0)
    white = (255,255,255)
    y = (255, 0,0)
    surf = pygame.surfarray.make_surface(canvas_for_visited)

    clock = pygame.time.Clock()
    done = False
    while not done:
        for event in pygame.event.get():  
            if event.type == pygame.QUIT:  
                done = True  
    
        # gameDisplay.fill(black)
        predec.pop((start_pos[0],start_pos[1]),None)
        for key,val in predec.items():    
            if not is_obstacle(key[0],key[1]) and not is_obstacle(val[0],val[1]):
                if (key != None and val != None):
                
                    pygame.draw.line(gameDisplay, white, (n*(val[0]),n*(display_height - 1 - val[1])),(m*(key[0]),m*(display_height - 1 - key[1])), 1)
                    pygame.display.flip()
                    pygame.time.wait(4)
        for i in result:
         if (i != None):
            pygame.draw.line(gameDisplay,y, (n*(i[0]),n*(display_height - 1 - i[1])),(m*(i[0]),m*(display_height - 1 - i[1])), 1)
            pygame.display.flip()
            pygame.time.wait(4)

        done = True
        
        pygame.time.delay(20)
        pygame.quit()
"""
[fn] A* algorithm

[in] - map
[in] - start
[in] - goal

This function takes the above inputs and find the shortest path using the dijkstra algorithm.


[out] returns the shortest path
"""
def astar(map, start, goal,wheel_rpm):
    
    pq = PriorityQueue() # OpenList
    theta = start[2]
    start = (start[0],start[1])
    pq.put(start, 0) # We add the start to our priority_queue
    predecessors = {start: None} # Closed List
    cost_to_come = {start: 0} # To store the cost2come values
    V = np.zeros((502,802,24)) # To store the visited nodes
    turtle_actions = []
    lin_ang = []
    

    while not pq.is_empty(): # Checking all the explored nodes
        current_cell = pq.get() # Popping the element based upon priority(cost)
        if chkgoal(current_cell,goal): # if this is goal we return the shortest path
            predecessors[goal] = current_cell
            print("Goal Reached")
            return get_path(predecessors, start, goal,turtle_actions,lin_ang)
        actions = action_space_rpm(wheel_rpm)
        for direction in ["(0,rpm1)","(rpm1,0)","(rpm1,rpm1)","(0,rpm2)","(rpm2,0)","(rpm2,rpm2)","(rpm1,rpm2)","(rpm2,rpm1)"]: # For all the possible actions 
            x,y,t,cost,dx,dy,dw= cost_rpm(current_cell,theta,actions[direction][0],actions[direction][1]) # We check for all the direction in the path using the above for loop
            neighbour = (int(np.round(x)),int(np.round(y)))
            if is_legal_pos(map,neighbour) and  neighbour not in cost_to_come and chk_duplicate(V,neighbour,t): # Checking if the neighbour is legal and not in closed list
                # Here we check whether the selected direction or action is a legal move
                # if yes we assign new cost and add it to the priority_queue based
                # upon the cost.
                # cost_to_come[neighbour] = cost_to_come[current_cell] + cost          
               
                cost_to_come[neighbour] = cost_to_come[current_cell] + cost
                f_value =cost_to_come[neighbour]+ heuristicEuclidean(current_cell, neighbour)
                pq.put(neighbour, f_value)
                predecessors[neighbour] = current_cell  
                turtle_actions.append(direction)
                lin_ang.append([dx,dy,(dw % 360)]) # We store the linear and angular velocity for the turtlebot
    print("Goal not reached")      
    return None
    


if __name__ == "__main__":
    
    # Here we call all our function calls in order to generate the shortest path
    global maze
    maze,obs_list = generate_map()
    fig, ax=plt.subplots()
    ax.imshow(maze[::-1,:,:])



   


    # select point from the given map output

    # start_pos,goal_pos = plt.ginput(2)
    # start_pos=(249- int(start_pos[1]),int(start_pos[0]),60)
    # goal_pos=(249 - int(goal_pos[1]),int(goal_pos[0]))
    start_pos = (20,20,30)
    goal_pos = (100,100)
    print("***      DIJKSTRA   ***")
    print("You have selected the start position as:",start_pos)
    print("You have selected the goal position as:",goal_pos)
    
    
    if is_obstacle(goal_pos[0],goal_pos[1]) or is_obstacle(start_pos[0],start_pos[1]) or check_obs_space(start_pos[0],start_pos[1]): 
         print("Please enter different start_pos and goal_pos away from the obstacle space")
    else:
        result,predec,turtle_actions,lin_ang = astar(maze, start_pos, goal_pos,(100,100))
        # Display the map with the explored nodes
        # predec.pop((start_pos[0],start_pos[1]),None)
        # for key,val in predec.items():
        #         if not is_obstacle(key[0],key[1]) and not is_obstacle(val[0],val[1]):
        #             cv2.line(maze,(key[0],key[1]),(val[0],val[1]),(0,0,255),1)
        # cv2.imshow("maze",maze[::-1,:,:])
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # for i in result:
        #         cv2.line(maze,(i[0],i[1]),(i[0],i[1]),(0,255,0),1)
        # cv2.imshow("maze",maze[::-1,:,:])
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # print("***      TURTLEBOT   ***")
        # print("The shortest path is:",result)
        
        
        
        
        
        
        print(len(predec))
        print("Result:",result)
        # print("Predecessors:",predec)
        print("The shortest path is:",len(result))
        display(predec,result,obs_list)
        






                                            
        
       