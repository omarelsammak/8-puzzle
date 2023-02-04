from queue import PriorityQueue
import copy
from cmath import sqrt
import time
#OMAR AHMED ELSAMMAK

def print_arr(arr):
    for i in range(3):
        print("---------------")
        for j in range(3):
            print("| " + str(arr[i][j]) + " |", end="")
        print()
    print("^^^^^^^^^^^^^^^" + "\n")

def arr_2_int(arr):
    num = 0
    for i in range(3):
        for j in range(3):
            power = (8 - (i*3 + j))
            num += (arr[i][j])*pow(10, power)
    return num

def int_2_arr(num):
    arr = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    for i in range(2, -1, -1):
        for j in range(2, -1, -1):
            if num == 0:
                break
            arr[i][j] = num % 10
            num = num // 10
    return arr

def move(arr,i,j,direction):
    if direction == 'down':
        arr[i][j], arr[i + 1][j] = arr[i + 1][j], arr[i][j]
    elif direction == 'up':
        arr[i][j], arr[i - 1][j] = arr[i - 1][j], arr[i][j]
    elif direction == 'right':
        arr[i][j], arr[i][j + 1] = arr[i][j + 1], arr[i][j]
    elif direction == 'left':
        arr[i][j], arr[i][j - 1] = arr[i][j - 1], arr[i][j]

def index_2d(myList, v):
    for i, x in enumerate(myList):
        if (v in x):
            return i, x. index(v)


def calculateheu(current,man_or_ec):
    curr = int_2_arr(current)
    goal = int_2_arr(goal_state)
    sumx = 0
    sumy = 0
    ec=0
    for i in range(0, 3):
        for j in range(0, 3):
            A = goal[i][j]
            tup = index_2d(curr, A)
            sumx += abs((tup[0]-i))
            sumy += abs((tup[1]-j))
            ec+=sqrt(pow(abs(sumx),2)+pow(abs(sumy),2))
        if  man_or_ec==1 :
            return (abs(sumx) + abs(sumy))
        elif  man_or_ec==0:
            return abs(ec)
def make_path(parent_map):
    path = []
    parent = parent_map.get(goal_state)
    path.append(goal_state)
    while parent != parent_map.get(parent):
        path.append(parent)
        parent = parent_map.get(parent)
    path.append(parent)
    return path


def is_goal(state):
    return state == goal_state


def getneighbors(current):
    current = int_2_arr(current)
    for i, x in enumerate(current):
        if 0 in x:
            i = i
            j = x.index(0)
            break
    neigbours = list()
    if i != 2:
        n1 = copy.deepcopy(current)
        move(n1, i, j,'down')
        neigbours.append(arr_2_int(n1))
    if i != 0:
        n1 = copy.deepcopy(current)
        move(n1, i, j,'up')
        neigbours.append(arr_2_int(n1))
    if j != 2:
        n1 = copy.deepcopy(current)
        move(n1, i, j,'right')
        neigbours.append(arr_2_int(n1))
    if j != 0:
        n1 = copy.deepcopy(current)
        move(n1, i, j,'left')
        neigbours.append(arr_2_int(n1))
    return neigbours
########################
def a_star(initial_state,man_or_ec):
    frontier = PriorityQueue()
    track_frontier = set()
    parent_map = dict()
    cost_map = dict()
    explored = set()

    frontier.put((calculateheu(initial_state, man_or_ec), initial_state))
    track_frontier.add(initial_state)
    parent_map = {initial_state: initial_state}
    cost_map = {initial_state: calculateheu(initial_state, man_or_ec)}

    while frontier.qsize() != 0:
        current_state = frontier.get()
        explored.add(current_state[1])
       # explored_astar+=1
        f = current_state[0]  # cost + heuristic
        current_cost = f - calculateheu(current_state[1], man_or_ec)
        explored.add(current_state[1])
        #explored_astar += 1
        if is_goal(current_state[1]):
            return parent_map
        else:
            neighbors = getneighbors(current_state[1])
            for i in neighbors:
                if i not in explored and i not in track_frontier:
                    frontier.put((current_cost + 1 + calculateheu(i, man_or_ec), i))
                    track_frontier.add(i)
                    parent_map.update({i: current_state[1]})
                    cost_map.update(
                        {i: current_cost + 1 + calculateheu(i, man_or_ec)})
                elif i not in explored:
                    if current_cost + 1 + calculateheu(i, man_or_ec) < cost_map.get(i):
                        frontier.put(
                            (current_cost + 1 + calculateheu(i, man_or_ec), i))
                        parent_map.update({i: current_state[1]})
                        cost_map.update(
                            {i: current_cost + 1 + calculateheu(i, man_or_ec)})
    return None
###################
def bfs(startState ):
    boardVisited = set()
    frontier =[]
    trackfrontier=set()
    parent_map = dict()
    frontier.append(startState)
    trackfrontier.add(startState)
    parent_map = dict({startState:startState})


    while len(frontier)>0:
        current = frontier.pop(0)
        boardVisited.add(current)
        #explored_bfs+=1
        if is_goal(current):
            return parent_map
        else :
            posiblePaths=getneighbors(current)
        for path in posiblePaths:
            if path not in boardVisited and path not in trackfrontier:
                frontier.append(path)
                trackfrontier.add(path)
                parent_map.update({path:current})
    return None
#####################
def dfs(startState):
    boardVisited = set()
    frontier =[]
    trackfrontier=set()
    parent_map = dict()
    frontier.append(startState)
    trackfrontier.add(startState)
    parent_map = dict({startState:startState})


    while len(frontier)>0:
        current = frontier.pop()
        boardVisited.add(current)
        #explored_dfs+=1
        if is_goal(current):
            return parent_map
        else :
            posiblePaths=getneighbors(current)
        for path in posiblePaths:
            if path not in boardVisited and path not in trackfrontier:
                frontier.append(path)
                trackfrontier.add(path)
                parent_map.update({path:current})
    return None
cost_bfs=0
cost_dfs=0
cost_manhattan=0
cost_eculidian=0
global explored_bfs,explored_dfs, explored_astar
explored_eculidian=0
explored_manhattan=0


initial_state = int(input("Enter initial state: ex 123456780"))
print("Initial state:")
print_arr(int_2_arr(initial_state))
goal_state = int(input("Enter goal state:\nex 123456708 \n"))
print("Goal state:")
goal = 123456780
print_arr(int_2_arr(goal_state))

#######################
print("using DFS:\n")
start = time.time()
dfs_parent_map = dfs(initial_state)
end = time.time()
dfstime=end-start
print("DFS finished\n")
if dfs_parent_map != None:
    dfs_path = make_path(dfs_parent_map)
    for i in range(len(dfs_path)-1, -1, -1):
        cost_dfs+=1
        array = int_2_arr(dfs_path[i])
        print_arr(array)
else:
    print("Unreachable\n\n")
#_____________________________________________________
print("using BFS:\n")
start = time.time()
bfs_parent_map = bfs(initial_state)
end = time.time()
bfstime=end-start
print("BFS finished\n")
if bfs_parent_map != None:
    bfs_path = make_path(bfs_parent_map)
    for i in range(len(bfs_path)-1, -1, -1):
        cost_bfs+=1
        array = int_2_arr(bfs_path[i])
        print_arr(array)
else:
    print("Unreachable\n\n")
# ----------------------------------------------------------------
print("starting A* Manhattan:\n")
start = time.time()
a_star_parent_map = a_star(initial_state,1)
end = time.time()
mantime=end-start
#explored_manhattan=explored_astar
explored_astar=0
print("stopping A*Manhattan\n")
if a_star_parent_map != None:
    a_star_path = make_path(a_star_parent_map)
    for i in range(len(a_star_path)-1, -1, -1):
        cost_manhattan+=1
        array = int_2_arr(a_star_path[i])
        print_arr(array)
else:
    print("Unreachable\n\n")
    #_____________________________________
print("starting A* Eculidian:\n")
start = time.time()
a_star_parent_map = a_star(initial_state,0)
end = time.time()
ectime=end-start
#explored_eculidian=explored_astar
print("stopping A* Eculidian\n")
if a_star_parent_map != None:
    a_star_path = make_path(a_star_parent_map)
    for i in range(len(a_star_path)-1, -1, -1):
        cost_eculidian+=1
        array = int_2_arr(a_star_path[i])
        print_arr(array)
else:
    print("Unreachable\n\n")

print("dfs time = ",dfstime)
print("cost dfs =", cost_dfs)
#print("explored nodes of dfs =",explored_dfs)
print("bfs time = ",bfstime)
print("cost bfs =", cost_bfs)
#print("explored nodes of bfs =",explored_bfs)
print("manhatan time = ",mantime)
print("cost manhattan =", cost_manhattan)
#print("explored nodes of manhattan =",explored_manhattan)
print("eculidian time = " ,ectime)
print("cost eculidian =", cost_eculidian)
#print("explored nodes of eculidian=",explored_eculidian)

