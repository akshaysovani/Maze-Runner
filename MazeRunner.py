## Maze Runner
# BFS
# DFS
# A star - Euclidean
# A star - Manhattan

import numpy as np
import random
import queue 
import math
#from tkinter import *

## Create a maze with given dimension and probability
def Create_Maze(dim,prob):
    
    maze = np.zeros((dim,dim))
    num_cells = dim * dim
    occupied_cells = int(prob * num_cells)
    print("Non blocked cells - %d\n"%(num_cells-occupied_cells))
    while occupied_cells > 0:
        row = random.randint(0,dim-1)
        column = random.randint(0,dim-1)
        while (row is 0 and column is 0) or (row is dim-1 and column is dim-1) or (maze[row][column] == 1):
            row = random.randint(0,dim-1)
            column = random.randint(0,dim-1)
        maze[row][column] = 1;
        occupied_cells -= 1

    ## Print the maze
    for row in range(0,dim):
        for column in range(0,dim):
            print("%d "%maze[row][column] ,end="")
        print()

    return maze

def GetChildNodesFromParent(maze,node,dim):
    child_nodes = []
    row = int(node / dim)
    column = node % dim
    assert (maze[row][column] == 0)
    if row > 0 and maze[row-1][column] != 1:
        child_nodes.append(((row - 1) * dim) + column)

    if column > 0 and maze[row][column-1] != 1:
        child_nodes.append((column - 1) + (row * dim))
    
    if row < dim - 1 and maze[row+1][column] != 1:
        child_nodes.append(((row + 1) * dim) + column)

    if column < dim - 1 and maze[row][column+1] != 1:
        child_nodes.append((column + 1) + (row * dim))

    return child_nodes



def GetChildNodesFromParentAstar(maze,node,dim,goal_x,goal_y,eu,man):
    child_nodes = []
    child_nodes_with_distance = []
    newlist = []
    row = int(node / dim)
    column = node % dim
    assert (maze[row][column] == 0)
    
    if row > 0 and maze[row-1][column] != 1:
        child_nodes.append(((row - 1) * dim) + column)

    if column > 0 and maze[row][column-1] != 1:
        child_nodes.append((column - 1) + (row * dim))
    
    if row < dim - 1 and maze[row+1][column] != 1:
        child_nodes.append(((row + 1) * dim) + column)

    if column < dim - 1 and maze[row][column+1] != 1:
        child_nodes.append((column + 1) + (row * dim))
    
    min_euclidean = 2*((dim**2)-1)
    eu_dict = {}
    if eu==1:
        for child in child_nodes:
            child_x = int(child / dim)
            child_y = child % dim
            euclidean = math.sqrt(((goal_x-child_x)**2) + ((goal_y-child_y)**2))
            newlist.append(euclidean)
            eu_dict.update({euclidean:child})
            
        newlist = sorted(newlist)
        for ele in newlist:
            child_nodes_with_distance.append([ele,eu_dict.get(ele)])
        return child_nodes_with_distance
        '''         
        return ordered_child_nodes    '''
            
        #return child_nodes
    
    elif man==1:
        for child in child_nodes:
            child_x = int(child / dim)
            child_y = child % dim
            euclidean = math.sqrt(((goal_x-child_x)**2)  +  ((goal_y-child_y)**2))
            if euclidean < min_euclidean:
                min_euclidean = euclidean
            return child_nodes



## BFS
def SolveMaze_BFS(maze,start,goal,dim): 
    print("\nBFS")
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_queue = queue.Queue()
    Path_queue.put([start])
    Visited = set()
    Done_find = False;
    path =[]
    while not Path_queue.empty() and not Done_find:
        path = Path_queue.get()
        node = path[-1]

        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodesFromParent(maze,node,dim):
                if node_child not in Visited:
                    path_1 = list(path)
                    path_1.append(node_child)
                    Path_queue.put(path_1)
                    if node_child == goal:
                        Done_find = True
                        path = path_1
                        break
        Visited.add(node)

    print("Explored cells - %d\n"%(len(Visited)+1))
    return path

## DFS
def SolveMaze_DFS(maze,start,goal,dim):
    print("\nDFS")
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_stack = list()
    Path_stack.insert(0,[start])
    Visited = set()
    Done_find = False;
    path =[]
    while Path_stack and not Done_find:
        path = Path_stack.pop(0)
        node = path[-1]

        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodesFromParent(maze,node,dim):
                if node_child not in Visited and not any(node_child in fringe for fringe in Path_stack):
                    path_1 = list(path)
                    path_1.append(node_child)
                    Path_stack.insert(0,path_1)
                    if node_child == goal:
                        Done_find = True
                        path = path_1
                        break
        Visited.add(node)

    print("Explored cells - %d\n"%(len(Visited)+1))
    return path



def SolveMaze_Astar_new(maze,start,goal,dim): 
    print("\nA star new")
    total_eu = math.sqrt(2*((399)**2))
    
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_queue = queue.PriorityQueue()
    Path_queue.put((total_eu,[start]))
    #Path_queue.put((priority,(0,total_eu)))
    Visited = set()
    Done_find = False;
    path =[]
    while not Path_queue.empty() and not Done_find:
        path = Path_queue.get()
        node = path[-1]

        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodesFromParentAstar(maze,node,dim):
                if node_child not in Visited:
                    path_1 = list(path)
                    path_1.append(node_child[1])
                    Path_queue.put((node_child[0],path_1))
                    if node_child == goal:
                        Done_find = True
                        path = path_1
                        break
        Visited.add(node)

    print("Explored cells - %d\n"%(len(Visited)+1))
    return path



















## A star euclidean distance
def SolveMaze_AstarEuclidean(maze,start,goal,dim):
    print("\nA star euclidean")
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_queue = queue.PriorityQueue()
    total_eu = math.sqrt(2*((399)**2))
    priority = 0
    Path_queue.put((priority,(0,total_eu)))
    Visited = set()
    Done_find = False;
    path = []
    eu=1
    man=0
    goal_x = (dim*dim) - 1 
    goal_y = (dim*dim) - 1
    while not Path_queue.empty() and not Done_find:
        to_sort_child_nodes = []
        sorted_child_nodes = []
        ordered_child_nodes = []
        path = Path_queue.get()
        node = path[-1][0]
        if node == goal:
            break
        elif node not in Visited:
            child_dict = GetChildNodesFromParentAstar(maze,node,dim,goal_x,goal_y,eu,man)
            if Path_queue.empty():
                for child,distance in child_dict.items():
                    to_sort_child_nodes.append(distance)
                sorted_child_nodes = sorted(to_sort_child_nodes)
                for dist in sorted_child_nodes:
                    for child,distance in child_dict.items():
                        repeating_record = 0
                        if dist==distance:
                            for item in ordered_child_nodes:
                                if item[0]==child and item[1]==dist:
                                    repeating_record = 1
                            if not repeating_record:        
                                ordered_child_nodes.append((child,dist))
                for each_tuple in ordered_child_nodes:
                    priority+=1
                    Path_queue.put((priority,each_tuple))
            
            else:
                
                
                
                for child,dist in child_dict.items():
                    put = 0
                    if child in Visited:
                        continue
                    samechild = 0
                    path_list = list(Path_queue.queue)
                    for i in path_list:
                        if child == i[1][0]:
                            samechild = 1
                            break
                    if samechild:
                        continue
                    if dist < path_list[0][1][1]:
                        new_priority_start = path_list[0][0] - 1
                        Path_queue.put((new_priority_start,(child,dist)))
                        continue
                    for q_item in path_list:
           
                        if dist < q_item[1][1]:
                            new_priority = q_item[0] - 0.05
                            Path_queue.put((new_priority,(child,dist)))
                            put = 1
                            break
                    if put == 1:
                        continue
                    new_priority_end = path_list[-1][0] + 1        
                    Path_queue.put((new_priority_end,(child,dist)))    
                
                
            '''for child,distance in child_dict:
                if child not in Visited:
                    
                    
            if node_child not in Visited:
                path_1 = list(path)
                path_1.append(node_child)
                Path_queue.put(path_1)
                if node_child == goal:
                    Done_find = True
                    path = path_1
                    break'''
        print('Node %d visited completely' % node)    
        Visited.add(node)
        print('current queue')
        print(list(Path_queue.queue))

    print("Explored cells - %d\n"%(len(Visited)+1))
    return path




## A star Manhattan distance
def SolveMaze_AstarManhattan():
    print("\nA star manhattan")




dim = 20
prob = 0.1
start = 0
goal = (dim*dim)-1
maze = Create_Maze(dim,prob)

'''
path = SolveMaze_BFS(maze,start,goal,dim)
if path[-1] != goal:
    print("\nUnsolvable maze")
else:
    for val in path:
        row = int(val / dim)
        column = val % dim
        print("(%d,%d) "%(row,column),end="")
    #print(path)

print()'''
'''
path = SolveMaze_DFS(maze,start,goal,dim)
if path[-1] != goal:
    print("\nUnsolvable maze")
else:
    for val in path:
        row = int(val / dim)
        column = val % dim
        print("(%d,%d) "%(row,column),end="")
print()
'''

path = SolveMaze_Astar_new(maze,start,goal,dim)
if path[-1] != goal:
    print("\nMaze unsolvable by A*")
else:
    for val in path:
        row = int(val / dim)
        column = val % dim
        print("(%d,%d) "%(row,column),end="")
print()

#path = SolveMaze_AstarEuclidean(maze,start,goal,dim)

#master = Tk()
#w = Canvas(master, width=500, height=500)
#w.pack()
#w.create_rectangle(50, 20, 500, 500)
#mainloop()
