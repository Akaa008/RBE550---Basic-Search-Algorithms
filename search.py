# Basic searching algorithms

# Class for each node in the grid
from ast import NodeTransformer


class Node:
    def __init__(self, row, col, is_obs, h):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.is_obs = is_obs  # obstacle?
        self.g = None         # cost to come (previous g + moving cost)
        self.h = h            # heuristic
        self.cost = None      # total cost (depend on the algorithm)
        self.parent = None    # previous node

def visit(grid):
    visited = []
    for i in range(len(grid)):
        row = []
        for j in range(len(grid[i])):
            row.append(grid[i][j])
        visited.append(row)
    return visited

def graph(grid):
    node_grid = []
    for i in range(len(grid)):
        ak = []
        for j in range(len(grid[i])):
            block = Node(i,j,False,True)
            if grid[i][j] == 0:
                block.g = float('inf')
                block.h = float('inf')
                block.is_obs = False
                block.cost = float('inf')
            ak.append(block)
        node_grid.append(ak)
    return node_grid




def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    
    path = []
    steps = 0
    found = False #reachd the goal position
        
    if not (goal[0] >= len(grid) or goal[0] < 0 or goal[1] >= len(grid) or start[0] >= len(grid) or start[0] < 0 or start[1] >= len(grid)):
        if ((grid[goal[0]][goal[1]]) == 1) or ((grid[start[0]][start[1]]) == 1):
            path = []
            steps = 0
            return path, steps
        visited = visit(grid)
        node_grid = graph(grid)

        start = node_grid[start[0]][start[1]]
        goal = node_grid[goal[0]][goal[1]]    
        
        queue = [start]
        visited[start.row][start.col] = 1

    
        while len(queue)>0:
            cell = queue.pop(0) #Slicing the first node
            steps = steps + 1 #step increment
            if cell.row == goal.row and cell.col == goal.col:
                found = True
                break
                
            #Check the right neighbour
            if cell.col+1 < len(grid[0]) and visited[cell.row][cell.col+1] == 0 and node_grid[cell.row][cell.col+1].is_obs == False:
                n_right = node_grid[cell.row][cell.col+1]
                n_right.parent = cell
                visited[n_right.row][n_right.col] = 1
                queue.append(n_right)
            
            #Check the down neighbour 
            if cell.row+1 < len(grid) and visited[cell.row+1][cell.col] == 0 and node_grid[cell.row+1][cell.col].is_obs == False:
                n_down = node_grid[cell.row+1][cell.col]
                n_down.parent = cell
                visited[n_down.row][n_down.col] = 1
                queue.append(n_down)
            
             #Check the left neighbour
            if cell.col-1 >= 0 and visited[cell.row][cell.col-1] == 0 and node_grid[cell.row][cell.col-1].is_obs == False:
                n_left = node_grid[cell.row][cell.col-1]
                n_left.parent = cell
                visited[n_left.row][n_left.col] = 1
                queue.append(n_left)
        
            #Check the up neighbour
            if cell.row-1 >= 0 and visited[cell.row-1][cell.col] == 0 and node_grid[cell.row-1][cell.col].is_obs == False:
                n_up = node_grid[cell.row-1][cell.col]
                n_up.parent = cell
                visited[n_up.row][n_up.col] = 1
                queue.append(n_up)

        #Path generation from parent
        while cell is not None:
            path.append([cell.row,cell.col])
            cell = cell.parent

        path = path[::-1]

        if found:
            print(f"It takes {steps} steps to find a path using BFS")
        else:
            print("No path found")
    return path, steps
    
def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    if not (goal[0] >= len(grid) or goal[0] < 0 or goal[1] >= len(grid) or start[0] >= len(grid) or start[0] < 0 or start[1] >= len(grid)):
        if ((grid[goal[0]][goal[1]]) == 1) or ((grid[start[0]][start[1]]) == 1):
            path = []
            steps = 0
            return path, steps
        start = Node(start[0],start[1],False,False)
        goal = Node(goal[0],goal[1],False,False)
        queue = [start]
        visited = visit(grid)

    
        while len(queue)>0:
            cell = queue.pop(-1)
            if visited[cell.row][cell.col] == 1:
                continue
            steps = steps + 1
            if cell.row == goal.row and cell.col == goal.col:
                found = True
                break
                
            visited[cell.row][cell.col] = 1
            if cell.row-1 >= 0 and visited[cell.row-1][cell.col] == 0:
                n_up = Node(cell.row-1,cell.col,False,False)
                n_up.parent = cell
                queue.append(n_up)

            if cell.col-1 >= 0 and visited[cell.row][cell.col-1] == 0:
                n_left = Node(cell.row,cell.col-1,False,False)
                n_left.parent = cell
                queue.append(n_left)
        
            if cell.row+1 < len(grid) and visited[cell.row+1][cell.col] == 0:
                n_down = Node(cell.row+1,cell.col,False,False)
                n_down.parent = cell
                queue.append(n_down)

            if cell.col+1 < len(grid[0]) and visited[cell.row][cell.col+1] == 0:
                n_right = Node(cell.row,cell.col+1,False,False)
                n_right.parent = cell
                queue.append(n_right)
               
        while cell is not None:
            path.append([cell.row,cell.col])
            cell = cell.parent
        path = path[::-1]

        if found:
            print(f"It takes {steps} steps to find a path using DFS")
        else:
            print("No path found")
    return path, steps


def dijkstra(grid, start, goal):
    '''Return a path found by Dijkstra alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False

    if not (goal[0] >= len(grid) or goal[0] < 0 or goal[1] >= len(grid) or start[0] >= len(grid) or start[0] < 0 or start[1] >= len(grid)):
        if ((grid[goal[0]][goal[1]]) == 1) or ((grid[start[0]][start[1]]) == 1):
            path = []
            steps = 0
            return path, steps
        visited = visit(grid)
        node_grid = graph(grid)


        node_grid[start[0]][start[0]].cost = 0
        queue = [node_grid[0][0]]
        start = node_grid[start[0]][start[1]]
        goal = node_grid[goal[0]][goal[1]]

        visited[start.row][start.col] = 1
    
        while len(queue)>0:
            queue.sort(key=lambda x:x.cost)
            cell = queue.pop(0)
            steps = steps + 1
            if cell.row == goal.row and cell.col == goal.col:
                found = True
                break
        

            if cell.col+1 < len(grid[0]) and visited[cell.row][cell.col+1] == 0 and node_grid[cell.row][cell.col+1].is_obs == False:
                n_right = node_grid[cell.row][cell.col + 1]
                n_right.parent = cell
                if n_right.cost > cell.cost + 1:
                    n_right.cost = cell.cost + 1
                    visited[n_right.row][n_right.col] = 1
                queue.append(n_right)


            if cell.row+1 < len(grid) and visited[cell.row+1][cell.col] == 0 and node_grid[cell.row+1][cell.col].is_obs == False:
                n_down = node_grid[cell.row+1][cell.col]
                n_down.parent = cell
                if n_down.cost > cell.cost + 1:
                    n_down.cost = cell.cost + 1
                    visited[n_down.row][n_down.col] = 1
                queue.append(n_down)
        

            if cell.col-1 >= 0 and visited[cell.row][cell.col-1] == 0 and node_grid[cell.row][cell.col-1].is_obs == False:
                n_left = node_grid[cell.row][cell.col - 1]
                n_left.parent = cell
                if n_left.cost > cell.cost + 1:
                    n_left.cost = cell.cost + 1
                visited[n_left.row][n_left.col] = 1
                queue.append(n_left)


            if cell.row-1 >= 0 and visited[cell.row-1][cell.col] == 0 and node_grid[cell.row-1][cell.col].is_obs == False:
                n_up = node_grid[cell.row-1][cell.col]
                n_up.parent = cell
                if n_up.cost > cell.cost + 1:
                    n_up.cost = cell.cost + 1
                visited[n_up.row][n_up.col] = 1
                queue.append(n_up)


        while cell is not None:
            path.append([cell.row,cell.col])
            cell = cell.parent
        path = path[::-1]


        if found:
            print(f"It takes {steps} steps to find a path using Dijkstra")
        else:
            print("No path found")
    return path, steps


def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    path = []
    steps = 0
    found = False
    
    if not (goal[0] >= len(grid) or goal[0] < 0 or goal[1] >= len(grid) or start[0] >= len(grid) or start[0] < 0 or start[1] >= len(grid)):
        if ((grid[goal[0]][goal[1]]) == 1) or ((grid[start[0]][start[1]]) == 1):
            path = []
            steps = 0
            return path, steps
        visited = visit(grid)
        node_grid = graph(grid)
    
        goal = Node(goal[0],goal[1],False,True) #initialize goal node
        queue = [node_grid[0][0]] #creating queue with just the starting grid
    
        node_grid[start[0]][start[0]].g = 0 #assign start cost to zero
        start = node_grid[start[0]][start[1]]
        visited[start.row][start.col] = 1
    
        start.h = abs(goal.row - start.row) + abs(goal.col - start.col) #calculae the heuristic distance from start to goal
    
        while len(queue):
            queue.sort(key=lambda x:x.cost) #arrange in ascending order
            cell = queue.pop(0)
            steps = steps + 1 #increment the steps
            if cell.row == goal.row and cell.col == goal.col: #check if node is goal position
                found = True
                break
        
            if cell.col+1 < len(grid[0]) and visited[cell.row][cell.col+1] == 0 and node_grid[cell.row][cell.col+1].is_obs == False: #right node check
                n_right = node_grid[cell.row][cell.col + 1] 
                n_right.parent = cell 
                if n_right.g > cell.g + 1: 
                    n_right.g = cell.g + 1
                    n_right.h = abs(goal.row - n_right.row) + abs(goal.col - n_right.col)  
                    n_right.cost = n_right.g + n_right.h

                visited[n_right.row][n_right.col] = 1
                queue.append(n_right)
            
            if cell.row+1 < len(grid) and visited[cell.row+1][cell.col] == 0 and node_grid[cell.row+1][cell.col].is_obs == False:
                n_down = node_grid[cell.row+1][cell.col]
                n_down.parent = cell
                if n_down.g > cell.g + 1:
                    n_down.g = cell.g + 1
                    n_down.h = abs(goal.row - n_down.row) + abs(goal.col - n_down.col)  
                    n_down.cost = n_down.g + n_down.h 
                visited[n_down.row][n_down.col] = 1
                queue.append(n_down)
            
            if cell.col-1 >= 0 and visited[cell.row][cell.col-1] == 0 and node_grid[cell.row][cell.col-1].is_obs == False:
                n_left = node_grid[cell.row][cell.col - 1]
                n_left.parent = cell
                if n_left.g > cell.g + 1:
                    n_left.g = cell.g + 1
                    n_left.h = abs(goal.row - n_left.row) + abs(goal.col - n_left.col)  
                    n_left.cost = n_left.g + n_left.h
                visited[n_left.row][n_left.col] = 1
                queue.append(n_left)

            if cell.row-1 >= 0 and visited[cell.row-1][cell.col] == 0 and node_grid[cell.row-1][cell.col].is_obs == False:
                n_up = node_grid[cell.row-1][cell.col]
                n_up.parent = cell
                if n_up.g > cell.g + 1:
                    n_up.g = cell.g + 1
                    n_up.h = abs(goal.row - n_up.row) + abs(goal.col - n_up.col) 
                    n_up.cost = n_up.h + n_up.g
                visited[n_up.row][n_up.col] = 1
                queue.append(n_up)

        while cell is not None:
            path.append([cell.row,cell.col])
            cell = cell.parent
        path = path[::-1]


        if found:
            print(f"It takes {steps} steps to find a path using A*")
        else:
            print("No path found")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
