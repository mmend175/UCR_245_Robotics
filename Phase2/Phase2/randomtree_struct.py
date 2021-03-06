
import random
import math
import numpy as np
from rtree import index

class Tree(object):
    def __init__(self, X):
        """
        Tree representation
        """
        p = index.Property()
        p.dimension = X.dimensions
        self.V = index.Index(interleaved=True, properties=p)  # vertices in an rtree
        self.V_count = 0
        self.E = {}  # edges in form E[child] = parent


def dist_between_points(a, b):
    """
    Return the Euclidean distance between two points
    """
    distance = sum(map(lambda a_b: (a_b[0] - a_b[1]) ** 2, zip(a, b)))

    return math.sqrt(distance)


class RandomTreeStruct(object):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc=0.01):
        """
        Template RRT planner
        X: Search Space
        Q: list of lengths of edges added to tree
        x_init: tuple, initial location
        x_goal: tuple, goal location
        max_samples: max number of samples to take
        r: resolution of points to sample along edge when checking for collisions
        prc: probability of checking whether there is a solution
        """
        self.X = X
        self.samples_taken = 0
        self.max_samples = max_samples
        self.Q = Q
        self.r = r
        self.prc = prc

        self.x_init = x_init
        self.x_goal = x_goal

        self.trees = []  # list of all trees
        self.add_tree()  # add initial tree

    def add_tree(self):
        """
        Create an empty tree and add to trees
        """
        self.trees.append(Tree(self.X))

    def add_vertex(self, tree, v):
        """
        Add vertex to corresponding tree
        """
        self.trees[tree].V.insert(0, v + v, v)
        self.trees[tree].V_count += 1  # increment number of vertices in tree
        self.samples_taken += 1  # increment number of samples taken

    def add_edge(self, tree, child, parent):
        """
        Add edge to corresponding tree
        """
        self.trees[tree].E[child] = parent

    def nearby(self, tree, x, n):
        """
        Return nearby vertices
        """
        return list(self.trees[tree].V.nearest(x, num_results=n, objects="raw"))


    def new_and_near(self, tree, x_new,copter_size):
        """
        Return a new steered vertex and the vertex in tree that is nearest
        """
        x_rand = self.X.sample_free(x_new)
        x_nearest = self.nearby(tree, x_rand, 1)[0] #Return vertex nearest to x_rand
        x_new = self.steer(x_nearest, x_rand, copter_size)
        # check if new point is in X_free and not already in V
        if not self.trees[0].V.count(x_new) == 0 or not self.X.obstacle_free(x_new):
            return None, None

        self.samples_taken += 1

        return x_new, x_nearest

    def connect_to_point(self, tree, x_a, x_b):
        """
        Connect vertex x_a in tree to vertex x_b
        Return: bool, True if able to add edge, False if prohibited by an obstacle
        """
        if self.trees[tree].V.count(x_b) == 0 and self.X.collision_free(x_a, x_b, self.r):
            self.add_vertex(tree, x_b)
            self.add_edge(tree, x_b, x_a)

            return True

        return False

    def can_connect_to_goal(self, tree):
        """
        Check if the goal can be connected to the graph

        Return: True if can be added, False otherwise
        """
        x_nearest = self.nearby(tree, self.x_goal, 1)[0]
        if self.x_goal in self.trees[tree].E and x_nearest in self.trees[tree].E[self.x_goal]:
            # tree is already connected to goal using nearest vertex
            return True

        if self.X.collision_free(x_nearest, self.x_goal, self.r):  # check if obstacle-free
            return True

        return False

    def get_path(self):
        """
        Return path through tree from start to goal
        :return: path if possible, None otherwise
        """
        if self.can_connect_to_goal(0):
            print("Can connect to goal")
            self.connect_to_goal(0)

            return self.reconstruct_path(0, self.x_init, self.x_goal)

        print("Could not connect to goal")

        return None

    def connect_to_goal(self, tree):
        """
        Connect x_goal to graph
        """
        x_nearest = self.nearby(tree, self.x_goal, 1)[0]
        self.trees[tree].E[self.x_goal] = x_nearest

    def steer(self, start, goal, distance):
        """
        Return a point in the direction of the goal, that is distance away from start
        """
        ab = np.empty(len(start), np.float)  # difference between start and goal
        for i, (start_i, goal_i) in enumerate(zip(start, goal)):
            ab[i] = goal_i - start_i

        ab = tuple(ab)
        zero_vector = tuple(np.zeros(len(ab)))

        ba_length = dist_between_points(zero_vector, ab)  # get length of vector ab
        unit_vector = np.fromiter((i / ba_length for i in ab), np.float, len(ab))
        # scale vector to desired length
        scaled_vector = np.fromiter((i * distance for i in unit_vector), np.float, len(unit_vector))
        steered_point = np.add(start, scaled_vector)  # add scaled vector to starting location for final point

        # if point is out-of-bounds, set to bound
        for dim, dim_range in enumerate(self.X.dimension_lengths):
            if steered_point[dim] < dim_range[0]:
                steered_point[dim] = dim_range[0]
            elif steered_point[dim] > dim_range[1]:
                steered_point[dim] = dim_range[1]

        return tuple(steered_point)

    def reconstruct_path(self, tree, x_init, x_goal):
        """
        Reconstruct path from start to goal
        """
        path = [x_goal]
        current = x_goal
        if x_init == x_goal:
            return path
        while not self.trees[tree].E[current] == x_init:
            path.append(self.trees[tree].E[current])
            current = self.trees[tree].E[current]
        path.append(x_init)
        path.reverse()

        return path

    def check_solution(self):
        # probabilistically check if solution found
        if self.prc and random.random() < self.prc:
            print("Checking if can connect to goal at", str(self.samples_taken), "samples")
            path = self.get_path()
            if path is not None:
                return True, path

        # check if can connect to goal after generating max_samples
        if self.samples_taken >= self.max_samples:
            return True, self.get_path()

        return False, None
