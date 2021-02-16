#Importing required libraries
import heapq
import numpy as np
import math
import time
import matplotlib.pyplot as plt

#Initializing the variables
start_time = time.time()
plotx = []
ploty = []
ox =[]
oy =[]
obstacle_space =[]
obs =[]
open_list = []
closed_list = []

#Takes the Start and Goal Node from the User
def Take_Input():
    start_x,start_y = list(map(int,(input("Please enter start x and y coordinate:- ").split())))
    goal_x,goal_y = list(map(int,(input("Please enter goal x and y coordinate:- ").split())))
    global start
    global goal
    start = (round(start_x),round(start_y))
    goal = (round(goal_x),round(goal_y))

#Creating the workspace
def generate_obstacles(robot_radius, clearance):
    margin = robot_radius + clearance
    workspace = np.zeros(shape=(int(303),int(203)))
    ox = []
    oy = []
    obstacle_space = []
    
    #create a border around the map (not obstacle)
    for i in range(round(302)):
            ox.append(i)
            oy.append(-1)
            obstacle_space.append([i,-1])

            ox.append(i)
            oy.append(201)
            obstacle_space.append([i,201])

    for i in range(round(202)):
            ox.append(-1)
            oy.append(i)
            obstacle_space.append([-1,i])

            ox.append(301)
            oy.append(i)
            obstacle_space.append([301,i])

    # add obstacles to the map
    for x in range(301):
        for y in range(201):
            # feasible workspace considering robot radius and clearance
            if (y - margin <= 0) or (x - margin <= 0) or (y - (200-margin) >= 0) or (x - (300-margin) >= 0):
                obstacle_space.append([x,y])
            # circle
            if (x - 225)**2 + (y - 150)**2 <= (25 + margin)**2:
                obstacle_space.append([x,y])
                
            # ellipse
            if ((x - 150)/(40 + margin))**2 + ((y - 100)/(20 + margin))**2 <= 1:
                obstacle_space.append([x,y])
            # rhombus
            if (5*y - 3*x + 5*(95 - margin) <= 0) and (5*y + 3*x - 5*(175 + margin) <= 0) and \
                    (5*y - 3*x + 5*(125 + margin) >= 0) and (5*y + 3*x - 5*(145 - margin) >= 0):
                obstacle_space.append([x,y])
                
            # rectangle
            if (5*y - 9*x - 5*(13 + margin) <= 0) and (65*y + 37*x - 5*1247 - 65*margin <= 0) and \
                    (5*y - 9*x + 5*(141 + margin) >= 0) and (65*y + 37*x - 5*1093 + 65*margin >= 0):
                obstacle_space.append([x,y])
                
            # polygon
            if (y <= 13*x - 140 + margin) and (y - x - 100 + margin >= 0) and \
                        (5*y + 7*x - 5*220 <= 0):
                obstacle_space.append([x,y])
            if (y - 185 - margin <= 0) and (5*y + 7*x - 5*(290 + margin) <= 0) and \
                    (5*y - 6*x - 5*(30 - margin) >= 0) and (5*y + 6*x - 5*(210 - margin) >= 0) and \
                        (5*y + 7*x - 5*(220 - margin) >= 0):
                obstacle_space.append([x,y])

    for i in obstacle_space:
        x = i[0]
        y = i[1]
        workspace[x][y] = 1

    #return obstacle_space
    x_obs=[col[0] for col in obstacle_space]
    y_obs=[col[1] for col in obstacle_space]
    plt.scatter(ox,oy,color = 'r')
    plt.scatter(x_obs,y_obs,color = 'r')
    Final_obs = []
    for i in range(302):
        Final_obs.append(workspace[i])
    return x_obs,y_obs,obstacle_space,Final_obs

#Set the action steps along with the cost2come
def  Make_Movements():
    steps = [[1,0,1],
             [0,1,1],
             [-1,0,1],
             [0,-1,1],
             [1,1,math.sqrt(2)],
             [1,-1,math.sqrt(2)],
             [-1,-1,math.sqrt(2)],
             [-1,1,math.sqrt(2)]]
    return steps

#Backtracking to get the path from the goal node to the final node
def Back_Track(my_List):
    backtrack = []
    l = len(my_List)
    current_pos = my_List[l-1][1]
    backtrack.append(current_pos)
    parent = my_List[l-1][2]
    while parent != None:
        for i in range(l):
            X = my_List[i]
            if X[1] == parent:
                parent = X[2]
                current_pos = X[1]
                backtrack.append(current_pos)
    return backtrack[::-1]

#Dijkstra Search
def dijkstra_algorithm(start,goal, workspace):
    start_node = (0,start,None)
    goal_node = (0,goal,None)
    movements =  Make_Movements()
    heapq.heappush(open_list,(start_node))
    workspace[start_node[1][0]][start_node[1][1]] = 1

    while len(open_list)>0:
        #Storing the visted nodes
        current_node = heapq.heappop(open_list)
        heapq.heappush(closed_list,current_node)
        plotx.append(current_node[1][0])
        ploty.append(current_node[1][1])

        #Visualizing the explored nodes
        if len(ploty)%1000 == 0:
            plt.plot(start[0], start[1], "yo")
            plt.plot(goal[0], goal[1], "yo")
            plt.plot(plotx,ploty, '.k')
            plt.pause(0.001)

        if current_node[1] == goal_node[1] :
            print('goal coordinates found')
            final_path = Back_Track(closed_list)
            return final_path
            
        #Exploring the nodes
        for new_position in movements:
            node_pos = (current_node[1][0] + new_position[0],
                             current_node[1][1] + new_position[1])
            node_position_cost = current_node[0] + new_position[2]
            node_parent = current_node[1]
            minx = 0
            miny = 0
            maxy = (len(workspace) - 1)
            maxx = (len(workspace[0]) -1)
            if node_pos[0] > maxy or node_pos[0] < miny or node_pos[1] > maxx or node_pos[1] < minx or workspace[node_pos[0]][node_pos[1]] != 0:
                continue
            workspace[node_pos[0]][node_pos[1]] = 1
            new_node = (node_position_cost,node_pos,node_parent)
            heapq.heappush(open_list,(new_node))


#Main program
Take_Input()
robot_radius = float(input("Enter radius of the rigid robot:- "))
clearance = float(input("Enter clearance value:- "))

x_obs,y_obs,obstacle_space,Final_obs = generate_obstacles(robot_radius, clearance)
if start in (zip(x_obs,y_obs)):
    print("Start node Invalid or in Obstacle Space")

elif goal in (zip(x_obs,y_obs)):
    print("Goal node Invalid or in Obstacle Space")

else:
    Generate_Path = dijkstra_algorithm(start,goal,Final_obs)
    plt.plot(plotx,ploty, '.k')
    if Generate_Path!= None:
        x = [x[0] for x in Generate_Path]
        y = [x[1] for x in Generate_Path]
        plt.plot(x,y,color = 'c',linewidth = 4)
        elapsed_time = time.time() - start_time
        print("Time elapsed-> ", elapsed_time)
    else:
        print("Path not found")
