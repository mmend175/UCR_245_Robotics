import math
import numpy as np
import time


class Node:
    def __init__(self,  parent=None,  position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self,  other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)


def motion_planner(init_pos,  goal_pos,  cMap):

    t0 = time.time()

    # Map Dimensions
    reset_loop = False
    xdim = cMap.shape[0]
    ydim = cMap.shape[1]
    zdim = cMap.shape[2]
    map_len = xdim*ydim*zdim

    ipos = (init_pos[0], init_pos[1], init_pos[2])
    gpos = (goal_pos[0], goal_pos[1], goal_pos[2])
    tot_dist = (gpos[0] - ipos[0]) + (gpos[1] - ipos[1]) + (gpos[2] - ipos[2])

    # Create Start and End Node
    start_node = Node(None,  ipos)
    start_node.g,  start_node.h,  start_node.f = 0,  0,  0
    end_node = Node(None,  gpos)
    end_node.g,  end_node.h,  end_node.f = 0,  0,  0

    # Initialize open and closed list
    open_list = []
    closed_list = set()

    # Add the start node
    open_list.append(start_node)

    flight_path = []

    # Loop until the end.
    while len(open_list) > 0:
        # print("Nodes Searched... Open Nodes: " + str(len(open_list)) + " | Closed Nodes: " + str(len(closed_list)))

        # Get current node
        current_node = open_list[0]
        current_index = 0

        for index,  item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        # print(current_node.position)
        # Pop current off the open list,  add it to the closed list
        open_list.pop(current_index)
        closed_list.add(current_node)

        cur_pos = current_node.position
        cur_dist = (gpos[0] - cur_pos[0]) + (gpos[1] - cur_pos[1]) + (gpos[2] - cur_pos[2])
        progress = 100 * (1 - cur_dist/tot_dist)
        print("Progress: %.2f%%" % progress)

        # If the goal has been found,  exit
        if current_node.position[0] == end_node.position[0] and \
                current_node.position[1] == end_node.position[1] and \
                current_node.position[2] == end_node.position[2]:
            current = current_node
            while current is not None:
                flight_path.append(current.position)
                current = current.parent
            flight_path.reverse()
            t1 = time.time()
            total_time = t1-t0
            return flight_path, total_time

        # Generate Children
        children = []   # Empty List each loop
        r = 3   # Distance to next searched node
        for new_position in [(0,   0, -r), (0,  -r,  0), (0,  -r,  -r),  (-r,  0,  0),  (-r,  0,  -r),
                             (-r, -r,  0), (-r, -r, -r), (0,  0,  r),    (0,  r,  0),   (0,  r,  r),
                             (r,  0,  0),  (r,  0,  r),  (r,  r,  0),    (r,  r,  r)]:
            # Get the node position
            node_position = (current_node.position[0] + new_position[0],
                             current_node.position[1] + new_position[1],
                             current_node.position[2] + new_position[2])

            # Make sure the node is in range
            if node_position[0] > xdim or node_position[0] < 0 or node_position[1] > ydim or node_position[1] < 0 \
                    or node_position[2] > zdim or node_position[2] < 0:
                continue

            # Make sure the move is valid (no obstacle)
            if cMap[node_position[0]-1,  node_position[1]-1,  node_position[2]-1] != 0:
                obs_node = Node(position=node_position)
                closed_list.add(obs_node)
                # current_node.h += 10
                continue

            # Create new node
            new_node = Node(current_node,  node_position)
            # Append child node
            children.append(new_node)

        # Loop through the children
        for child in children:

            # continue if child is already on the closed list
            if child in closed_list:
                # reset_loop = False
                # if child == closed_child:
                #     reset_loop = True
                continue
            # if reset_loop:
            #     continue

            # Find g,  h,  f values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0])**2) + \
                      ((child.position[1] - end_node.position[1])**2) + \
                      ((child.position[2] - end_node.position[2])**2)
            child.f = child.g + child.h

            # continue if the child is already on the open list and is greater than the current g
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    reset_loop = True
                    continue
            # if reset_loop:
            #     continue

            # Add the child to the open list
            open_list.append(child)
