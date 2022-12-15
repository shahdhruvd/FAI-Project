#! /usr/bin/env python

import rospy
from math import sqrt
from algorithms.neighbors import find_neighbors


def greedy(initialNode, goalNode, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):
  ''' 
  Performs Greedy shortest path algorithm search on a costmap with a given start and goal node
  
  '''

  def calculate_euclidean_distance(initialNode, goalNode, width):
    """ Heuristic Function for A Star algorithm"""
    x1 = initialNode % width
    x2 = goalNode % width
    y1 = int(initialNode / width)
    y2 = int(goalNode / width)
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

  queue = []

  parentList = dict()

  travelCost = dict()

  costIncludingHeuristics = dict()

  travelCost[initialNode] = 0
  costIncludingHeuristics[initialNode] = 0

  queue.append([initialNode, 0])

  shortestPath = []

  visited = set()

  pathFound = False
  rospy.loginfo('Greedy: Done with initialization')


  while queue:

    # get node with lowest cost
    queue.sort(key = lambda x: x[1]) 
    # extract the first node (the one with the lowest 'cost' value)
    currentNode = queue.pop(0)[0]

    grid_viz.set_color(currentNode,"red")

    visited.add(currentNode)

    if currentNode == goalNode:
      pathFound = True
      break

    # Get neighbors of currentNode
    neighbors = find_neighbors(currentNode, width, height, costmap, resolution)

    for neighbor, neighborCost in neighbors:

      # Check if the neighbor has already been visited
      if neighbor in visited:
        continue

      # calculate cost of neighbour considering it is reached through currentNode
      cost = calculate_euclidean_distance(neighbor, goalNode, width)
      #actualCost = cost + calculate_euclidean_distance(neighbor, goalNode, width)

      # Check if the neighbor is in queue
      in_queue = False
      for index, node in enumerate(queue):
        if node[0] == neighbor:
          in_queue = True
          break

      if in_queue:
        if cost < costIncludingHeuristics[neighbor]:
          #travelCost[neighbor] = cost
          costIncludingHeuristics[neighbor] = cost
          parentList[neighbor] = currentNode
          queue[index] = [neighbor, cost]

      else:
        #travelCost[neighbor] = cost
        costIncludingHeuristics[neighbor] = cost
        parentList[neighbor] = currentNode
        # Add neighbor to queue
        queue.append([neighbor, cost])

        grid_viz.set_color(neighbor,'green')

  if pathFound:
      node = goalNode
      shortestPath.append(goalNode)
      while node != initialNode:
          shortestPath.append(node)
          node = parentList[node]
  shortestPath = shortestPath[::-1]
  rospy.loginfo('Greedy: Done traversing nodes in open_list')

  if not pathFound:
    rospy.logwarn('Greedy: No path found!')
    return shortestPath

  return shortestPath, None
