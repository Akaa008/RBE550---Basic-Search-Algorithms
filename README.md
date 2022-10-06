# RBE550---Basic-Search-Algorithms
  1.Grid - A nested list with datatype int
  2. Start - Starting node of the map
  3. Goal - Target node to be reached in the map

The variables used here are:
  1. Visited - Nested list to check if the grid is explored
  2. Node_grid - A linked list used to update initialized cost, g, h , obstacle
  3. Queue - List of nodes to be explored next
  4. Cell - The current cell that transverses through the while loop

The functions defined are defined:
1. Visited function goes through the grid of the map that is given as the input and stores the
values of the grids as a nested list for updating the cells visited in the algorithm

2. Graph function, we create a node called “Block”. We used the node “Block” to initialize
the tentative cost i.e., g , the heuristic i.e., h , the total cost i.e., cost and obstacle check
i.e., is_obs. We will be appending them into node_grid which is a linked list

The path for the algorithms are traced in the order of Right - Down - Left - Up

BFS Pseudocode:
Function visited:
  For all values in grid:
    Duplicate the grid as nested list
Function graph:
  Go through each cells of the grid
Create a node called block
If grid value = 0:
Set as no obstacle in the block
Append the block to node_grid
Function BFS:
Initialize start and goal nodes
visited(start.row,start.col) = 1
While queue is not empty:
Cell = Pop the first node from queue
Increment step
If cell = goal
Break
If neighboring cell < length, not visited, no obstacle (in order of right, down,left,right)
Update neighboring node
Update parent node
Set this node as visited
Append to queue
While cell is not empty:
Dequeue from parent cell to path list
DFS:
The DFS search algorithm is similar to BFS but is a bit different such that
“stacking” is used here instead of queue. The concept of First In Last Out (FILO) is
used. The neighbors of the starting node are checked one by one. In the first level
neighboring node, the second level neighboring node is inspected and from the second
level node, the third level neighboring node is inspected till the end of the node and
backtracks to the first level node and repeats the same for the neighboring node until
the goal node is reached. The difference between the BFS and DFS code is that in
DFS, for stacking and last in first out, the neighbours are referred in the reverse order
i.e., up,left,down and rightPseudocode:
Function visited:
For all values in grid:
Duplicate the grid as nested list
Function DFS:
Initialize start and goal nodes
Queue = [start]
visited(start.row,start.col) = 1
While queue is not empty:
Cell = Pop the first node from queue
Increment step
If cell = goal
Break
If neighbouring cell < length, not visited, no obstacle (in order of up,left,down,right)
Update neighbouring node
Update parent node
Set this node as visited
Append to queue
While cell is not empty:
Dequeue from parent cell to path list
Dijkstra:
The Dijkstra algorithm is similar to BFS but with a difference that this algorithm
uses cost and priority queue. The priority queue is sorted based on the cost of reaching
a particular node.
Similar to BFS, the cost of each node is initialized to infinity along with obstacle
initialization. Start and goal nodes are initialized and the queue is set with start node.
The start node is assigned to be visited. The queue in the while loop is first arranged in
an ascending order so that the node with the least cost is popped out first from the
queue. And similar to the other search algorithms, the neighbors are explored and the
cost of each of the explored neighbor nodes are updated and appended to the queue.Pseudo code:
Function visited:
For all values in grid:
Duplicate the grid as nested list
Function graph:
Go through each cells of the grid
Create a node called block
If grid value = 0:
Set as no obstacle in the block
Initialize the cost to infinity for all cells
Append the block to node_grid
Function Dijkstra:
Initialize start and goal nodes
visited(start.row,start.col) = 1
Set queue to start node
While queue is not empty:
Arrange the queue in ascending order
Cell = Pop the first node from queue
Increment step
If cell = goal
Break
If neighboring cell < length, not visited, no obstacle (in order of right, down,left,right)
Update neighboring node
Update parent node
Set this node as visited
Node cost = cell cost + 1
Append to queue
While cell is not empty:
Dequeue from parent cell to path list
A* :
Code explanation:
The A* search algorithm, also called as “Best first search” algorithm is similar to
Dijkstra but the difference is that an additional metric called the heuristic is used here.
Unlike Dijkstra algorithm, A* uses heuristics that gives priority to better paths than
Dijkstra where all possible paths are explored. The cost function for the A* algorithm is
shown below,f(x) = g(x) + h(x)
Where,
f(x) - total cost
g(x) - cost to current node from start node
h(x) - heuristic i.e., distance from current node to goal
Heuristics is calculated using manhattan distance, |x_current - x_goal| + |y_current -
y_goal|
Similar to the previous search algorithms, additional g and h nodes are initialized to
infinity in the graph function. The manhattan distance from the start to goal node is
calculated as the initial heuristics. The priority queue is initialized and is sorted in terms
of least cost. The first index of the queue is taken as the cell variable and is checked for
goal position and the neighboring nodes are explored and the total cost for each node is
calculated and appended to the queue. The current cell is assigned as the parent node
for the neighboring node. As the queue is popped in the while loop, the node with the
least cost is popped and the new nodes are explored. The visited nodes when
encountered when exploring neighbors of new nodes are skipped.
Pseudo-code:
Function visited:
For all values in grid:
Duplicate the grid as nested list
Function graph:
Go through each cells of the grid
Create a node called block
If grid value = 0:
Set as no obstacle in the block
Initialize the cost,g,h to infinity for all cells
Append the block to node_grid
Function Dijkstra:
Initialize start and goal nodes
visited(start.row,start.col) = 1
Set queue to start node
While queue is not empty:
Arrange the queue in ascending order of total cost
Cell = Pop the first node from queue
Increment step
If cell = goal
BreakIf neighboring cell < length, not visited, no obstacle (in order of right, down,left,right)
Update neighboring node
Update parent node
Set this node as visited
Node cost, g = cell cost + 1
Calculate h i.e., manhattan distance
Total cost = g + h
Append to queue
While cell is not empty:
Dequeue from parent cell to path list
OUTPUT:
BFS:
DFS:DJIKSTRA:
A*:
The number of steps for each algorithm is:
It is observed that BFS and DIJKSTRA take the same number of steps to find the
shortest path to the goal. The cost to transition from start to the next cell is taken to be
the 1 for both the algorithms.
For DFS, the concept stacking is used whereas queuing is used in BFS. So, the path
obtained is different to the path obtained for BFS
For A*, the steps taken to reach the goal is the least compared to other algorithms as it
uses heuristics to choose the better path to reach the goal position
Boundary Conditions:
The boundary conditions for the search algorithm are implemented. The conditions
eliminated are:
1. Start and goal position out of grid limit
2. Start and goal position on obstacleThe pseudocode for eliminating edge conditions:
Def BFS/DFS/Dijkstra/A*:
If not start and goal position out of limits:
If start and goal is on obstacle:
Return empty path and 0 steps
Continue with search algorithm code
1. START POSITION OUT OF GRID2. GOAL POSITION OUT OF GRID3. START POSITION ON OBSTACLE
Considering the start position to be on the obstacle at (1,1)4. GOAL POSITION ON OBSTACLE
Considering the goal position to be on the obstacle at (1,1)
