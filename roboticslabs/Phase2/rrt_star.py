from operator import itemgetter
from randomtree_struct import RandomTreeStruct
import math

def dist_between_points(a, b):

    #Return the Euclidean distance between two points

    distance = sum(map(lambda a_b: (a_b[0] - a_b[1]) ** 2, zip(a, b)))

    return math.sqrt(distance)

def cost_to_go(a: tuple, b: tuple) -> float:
    """
    #return: estimated segment_cost-to-go from a to b
    """
    return dist_between_points(a, b)


def path_cost(E, a, b):
    """
    #Cost of the unique path from x_init to x
    #return: segment_cost of unique path from x_init to x
    """
    cost = 0
    while not b == a:
        p = E[b]
        cost += dist_between_points(b, p)
        b = p

    return cost


def segment_cost(a, b):
    #return: segment_cost function between a and b

    return dist_between_points(a, b)


class RRTStar(RandomTreeStruct):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc=0.01, rewire_count=None):
        """
        RRT* Search
        X: Search Space
        Q: list of lengths of edges added to tree
        x_init: tuple, initial location
        x_goal: tuple, goal location
        max_samples: max number of samples to take
        r: resolution of points to sample along edge when checking for collisions
        prc: probability of checking whether there is a solution
        rewire_count: number of nearby vertices to rewire
        """
        super().__init__(X, Q, x_init, x_goal, max_samples, r, prc)
        self.rewire_count = rewire_count if rewire_count is not None else 0
        self.c_best = float('inf')  # length of best solution thus far

    def get_nearby_vertices(self, tree, x_init, x_new):
        """
        Get nearby vertices to new vertex and their associated path costs from the root of tree
        as if new vertex is connected to each one separately.
        return: list of nearby vertices and their costs, sorted in ascending order by cost
        """
        X_near = self.nearby(tree, x_new, self.current_rewire_count(tree))
        L_near = [(path_cost(self.trees[tree].E, x_init, x_near) + segment_cost(x_near, x_new), x_near) for
                  x_near in X_near]
        # noinspection PyTypeChecker
        L_near.sort(key=itemgetter(0))

        return L_near

    def rewire(self, tree, x_new, L_near):
        """
        Rewire tree to shorten edges if possible
        Only rewires vertices according to rewire count
        """
        for c_near, x_near in L_near:
            curr_cost = path_cost(self.trees[tree].E, self.x_init, x_near)
            #print(curr_cost)
            tent_cost = path_cost(self.trees[tree].E, self.x_init, x_new) + segment_cost(x_new, x_near)
            if tent_cost < curr_cost and self.X.collision_free(x_near, x_new, self.r):
                print("NEW")
                print(self.trees[tree].E[x_near])
                break;
                self.trees[tree].E[x_near] = x_new

    def connect_shortest_valid(self, tree, x_new, L_near):
        """
        Connect to nearest vertex that has an unobstructed path
        """
        # check nearby vertices for total cost and connect shortest valid edge
        for c_near, x_near in L_near:
            if c_near + cost_to_go(x_near, self.x_goal) < self.c_best and self.connect_to_point(tree, x_near, x_new):
                break

    def current_rewire_count(self, tree):
        """
        Return rewire count

        """
        # if no rewire count specified, set rewire count to be all vertices
        if self.rewire_count is None:
            return self.trees[tree].V_count

        # max valid rewire count
        return min(self.trees[tree].V_count, self.rewire_count)

    def rrt_star(self,copter_size):
        """
        :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
        """
        self.add_vertex(0, self.x_init)
        self.add_edge(0, self.x_init, None)
        x_new=self.x_init
        while True:
            for q in self.Q:  # iterate over different edge lengths
                for i in range(q[1]):  # iterate over number of edges of given length to add
                    x_new, x_nearest = self.new_and_near(0,x_new,copter_size)
                    if x_new is None:
                        continue

                    # get nearby vertices and cost-to-come
                    L_near = self.get_nearby_vertices(0, self.x_init, x_new)

                    # check nearby vertices for total cost and connect shortest valid edge
                    self.connect_shortest_valid(0, x_new, L_near)
                    #if x_new in self.trees[0].E:
                        #rewire tree

                        #self.rewire(0, x_new, L_near)

                    solution = self.check_solution()
                    if solution[0]:
                        return solution[1]
