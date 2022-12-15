#! /usr/bin/env python

import rospy
from math import sqrt
from algorithms.neighbors import find_neighbors


def astar(initialNode, goalNode, width, height, costmap, resolution, origin, grid_viz, previous_plan_variables):
  ''' 
  Performs A Star shortest path algorithm search on a costmap with a given start and goal node
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

  queue.append([initialNode, 0 + calculate_euclidean_distance(initialNode, goalNode, width)])

  shortestPath = []

  visited = set()

  pathFound = False
  rospy.loginfo('A Star: Done with initialization')


  while queue:

    # get node with lowest cost
    queue.sort(key = lambda x: x[1]) 
    # extract the first node (the one with the lowest 'cost' value)
    currentNode = queue.pop(0)[0]

    grid_viz.set_color(currentNode,"pale yellow")

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
      cost = travelCost[currentNode] + neighborCost
      actualCost = cost + calculate_euclidean_distance(neighbor, goalNode, width)

      # Check if the neighbor is in queue
      in_queue = False
      for index, node in enumerate(queue):
        if node[0] == neighbor:
          in_queue = True
          break

      if in_queue:
        if actualCost < costIncludingHeuristics[neighbor]:
          travelCost[neighbor] = cost
          costIncludingHeuristics[neighbor] = actualCost
          parentList[neighbor] = currentNode
          queue[index] = [neighbor, actualCost]

      else:
        travelCost[neighbor] = cost
        costIncludingHeuristics[neighbor] = actualCost
        parentList[neighbor] = currentNode
        # Add neighbor to queue
        queue.append([neighbor, actualCost])

        grid_viz.set_color(neighbor,'orange')

  if pathFound:
      node = goalNode
      shortestPath.append(goalNode)
      while node != initialNode:
          shortestPath.append(node)
          node = parentList[node]
  shortestPath = shortestPath[::-1]
  rospy.loginfo('AStar: Done reconstructing path')

  if not pathFound:
    rospy.logwarn('AStar: No path found!')
    return shortestPath

  return shortestPath, None