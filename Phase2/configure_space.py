import math
import random
import uuid

import numpy as np
from rtree import index


def obstacle_generator(obstacles):
    """
    Add obstacles to r-tree
    """
    for obstacle in obstacles:
        yield (uuid.uuid4(), obstacle, obstacle)


def dist_between_points(a, b):
    """
    #return: Euclidean distance between a and b
    """
    distance = sum(map(lambda a_b: (a_b[0] - a_b[1]) ** 2, zip(a, b)))

    return math.sqrt(distance)


class ConfigureSpace(object):
    def __init__(self, dimension_lengths, O=None):
        """
        Initialize Search Space
        """
        # sanity check
        if len(dimension_lengths) < 2:
            raise Exception("Must have at least 2 dimensions")
        self.dimensions = len(dimension_lengths)  # number of dimensions
        # sanity checks
        if any(len(i) != 2 for i in dimension_lengths):
            raise Exception("Dimensions can only have a start and end")
        if any(i[0] >= i[1] for i in dimension_lengths):
            raise Exception("Dimension start must be less than dimension end")
        self.dimension_lengths = dimension_lengths  # length of each dimension
        p = index.Property()
        p.dimension = self.dimensions
        if O is None:
            self.obs = index.Index(interleaved=True, properties=p)
        else:
            # r-tree representation of obstacles
            # sanity check
            if any(len(o) / 2 != len(dimension_lengths) for o in O):
                raise Exception("Obstacle has incorrect dimension definition")
            if any(o[i] >= o[int(i + len(o) / 2)] for o in O for i in range(int(len(o) / 2))):
                raise Exception("Obstacle start must be less than obstacle end")
            self.obs = index.Index(obstacle_generator(O), interleaved=True, properties=p)

    def obstacle_free(self, x):
        """
        Check if a location resides inside of an obstacle

        """
        return self.obs.count(x) == 0

    def sample_free(self,x_new):
        """
        Sample a location within X_free
        """

        while True:  # sample until not inside of an obstacle
            x = self.sample()
            if self.obstacle_free(x):
                return x

    def collision_free(self, start, end, r):
        """
        Check if a line segment intersects an obstacle
        """
        dist = dist_between_points(start, end)
        # divide line between points into equidistant points at given resolution
        dim_linspaces = [np.linspace(s_i, e_i, int(math.ceil(dist / r))) for s_i, e_i in zip(start, end)]

        coll_free = all(map(self.obstacle_free, zip(*dim_linspaces)))

        return coll_free

    def sample(self):
        """
        Return a random location within X
        """
        x = np.empty(len(self.dimension_lengths), np.float)
        for dimension in range(len(self.dimension_lengths)):

            x[dimension] = random.uniform(self.dimension_lengths[dimension][0], self.dimension_lengths[dimension][1])

        return tuple(x)
