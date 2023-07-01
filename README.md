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
