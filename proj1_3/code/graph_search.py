from heapq import heappush, heappop  # Recommended.
import numpy as np
import math
import time

from flightsim.world import World
from proj1_3.code.occupancy_map import OccupancyMap  # Recommended.


def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """

    class Node:
        # tracks the index, parent and costs of each node in the graph
        def __init__(self, index, parent, gcost, hcost):
            self.index = index
            self.parent = parent  # index of parent
            self.g = gcost
            self.h = hcost

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)

    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    # Start with unexplored points
    unexploredPointExists = 1
    # makes sure path isn't trivial before starting loop
    if (start_index != goal_index):
        notatgoal = 1
        curr_ind = start_index
        curr_node = Node(start_index, start_index, 0, math.sqrt(sum((goal - start) ** 2)))
        curr_pos = occ_map.index_to_metric_center(curr_ind)
        curr_key = tuple(curr_ind)

        goal_cent_pos = occ_map.index_to_metric_center(goal_index)
    else:
        notatgoal = 0
    # can't leave visited empty, or the np.all comparison doesn't work
    visited_nodes = {tuple(start_index): curr_node}
    unvisited_nodes = {}
    unvisited_PQ = []

    time_neighbor = 0
    time_occcheck = 0
    time_occcheck2 = 0
    time_occcheck3 = 0

    # precalculate g addition values
    neighborList = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1],
                             [-1, 0, 0], [0, -1, 0], [0, 0, -1],
                             [1, 1, 0], [0, 1, 1], [1, 0, 1],
                             [-1, -1, 0], [0, -1, -1], [-1, 0, -1],
                             [-1, 1, 0], [0, -1, 1], [-1, 0, 1],
                             [1, -1, 0], [0, 1, -1], [1, 0, -1],
                             [1, 1, 1], [-1, -1, -1], [-1, 1, 1],
                             [1, -1, 1], [1, 1, -1], [-1, -1, 1],
                             [-1, 1, -1], [1, -1, -1]]) + 1
    costToNeighbor = np.zeros((26, 1))
    refpos = occ_map.index_to_metric_center(np.array([1, 1, 1]))

    for i in range(0, 26):
        neigh_pos = occ_map.index_to_metric_center(neighborList[i, :])
        costToNeighbor[i] = math.sqrt(sum((neigh_pos - refpos) ** 2))

    # main algorithm
    while unexploredPointExists:
        time_start_neighbor = time.time()
        # get valid neighbors
        neighborList = curr_ind + np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1],
                                            [-1, 0, 0], [0, -1, 0], [0, 0, -1],
                                            [1, 1, 0], [0, 1, 1], [1, 0, 1],
                                            [-1, -1, 0], [0, -1, -1], [-1, 0, -1],
                                            [-1, 1, 0], [0, -1, 1], [-1, 0, 1],
                                            [1, -1, 0], [0, 1, -1], [1, 0, -1],
                                            [1, 1, 1], [-1, -1, -1], [-1, 1, 1],
                                            [1, -1, 1], [1, 1, -1], [-1, -1, 1],
                                            [-1, 1, -1], [1, -1, -1]])
        for i in range(0, 26):
            neighbor = neighborList[i]
            # if on the map, unvisited, and unoccupied
            time_start_2 = time.time()
            if occ_map.is_valid_index(neighbor):
                time_start = time.time()
                neighbor_key = tuple(neighbor)
                time_occcheck += time.time() - time_start

                if ~(neighbor_key in visited_nodes):
                    # if unoccupied
                    if ~occ_map.is_occupied_index(neighbor):
                        # check if you've hit the goal
                        time_occcheck3 += time.time() - time_start_2
                        time_start = time.time()
                        if np.all(neighbor == goal_index):
                            # Cost to come
                            g = visited_nodes[curr_key].g + costToNeighbor[i]
                            # if we haven't found the goal before
                            if notatgoal:
                                notatgoal = 0
                                goal_node = Node(neighbor, curr_ind, g, 0)
                            # if we have found the goal before, see if the new path is closer than the old one
                            elif g < goal_node.g:
                                goal_node = Node(neighbor, curr_ind, g, 0)
                        # calc cost, add to unvisited list
                        else:
                            # Cost to come
                            g = visited_nodes[curr_key].g + costToNeighbor[i]
                            # Plus the Heuristic, if A*
                            if astar:
                                neig_pos = occ_map.index_to_metric_center(neighbor)
                                h = math.sqrt(sum((goal - neig_pos) ** 2))
                            else:
                                h = 0
                            if neighbor_key in unvisited_nodes:
                                if unvisited_nodes[neighbor_key].g + unvisited_nodes[neighbor_key].h > g + h:
                                    # if the new node gets there faster than the previous one, update the cost/parent
                                    unvisited_nodes[neighbor_key].g = g
                                    unvisited_nodes[neighbor_key].h = h
                                    unvisited_nodes[neighbor_key].index = neighbor
                                    unvisited_nodes[neighbor_key].parent = curr_ind
                                    heappush(unvisited_PQ, (g + h, neighbor_key))
                            else:
                                # add the neighbor to the list of unvisited nodes, and the the priority queue
                                unvisited_nodes[neighbor_key] = Node(neighbor, curr_ind, g, h)
                                heappush(unvisited_PQ, (g + h, neighbor_key))
                            time_occcheck2 += time.time() - time_start

        time_neighbor += (time.time() - time_start_neighbor)

        # All neighbors added, so move to next node if you aren't done
        if (unvisited_PQ != []) & notatgoal:
            # select the lowest cost node
            temp = heappop(unvisited_PQ)

            # if we got a node that has an updated version, skip the old version
            while unvisited_nodes[temp[1]].g + unvisited_nodes[temp[1]].h < temp[0]:
                # make sure we haven't emptied unvisited_PQ in the mean time
                if not unvisited_PQ:
                    unexploredPointExists = 0
                    break
                temp = heappop(unvisited_PQ)
            # if we still have a valid next point, after clearing the old duplicates, update the current point
            if unexploredPointExists:
                curr_node = unvisited_nodes[temp[1]]
                curr_ind = curr_node.index
                curr_pos = occ_map.index_to_metric_center(curr_ind)
                curr_key = tuple(curr_ind)

                # update the visited list
                visited_nodes[curr_key] = curr_node
        # if you've hit the goal, check the unvisited options for faster options
        elif unexploredPointExists & ~notatgoal:
            # update the visited list
            visited_nodes[curr_key] = curr_node
            # select the lowest cost node
            temp = heappop(unvisited_PQ)
            # if there are still shorter paths to be explored, keep looking as normal
            if temp[0] < goal_node.g:
                curr_node = unvisited_nodes[temp[1]]
                curr_ind = curr_node.index
                curr_pos = occ_map.index_to_metric_center(curr_ind)
                curr_key = tuple(curr_ind)
                # update the visited list
                visited_nodes[curr_key] = curr_node
            # if all the remaining unexplored nodes are further than the goal is currently, we don't care about them
            else:
                unexploredPointExists = 0
        else:
            unexploredPointExists = 0
    if notatgoal:
        return None
    else:
        # We're going through the list backwards, appending each parent, then the start
        path = [goal]
        curr_node = goal_node
        curr_node = visited_nodes[tuple(curr_node.parent)]

        while ~np.all(curr_node.parent == start_index):
            path.append(occ_map.index_to_metric_center(curr_node.index))
            # print("position: ",occ_map.index_to_metric_center(curr_node.index)," g: ",curr_node.g," h: ",curr_node.h)
            curr_node = visited_nodes[tuple(curr_node.parent)]
        # add the start location, flip, and convert to a Nx3 matrix
        path.append(start)
        path.reverse()
        path = np.asarray(path)
        print("Time in Neighbors: ", time_neighbor)

        return path
