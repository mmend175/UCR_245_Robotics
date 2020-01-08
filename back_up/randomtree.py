from randomtree_struct import RandomTreeStruct


class Randomtree(RandomTreeStruct):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc=0.01):
        """
        :param X: Search Space
        :param Q: list of lengths of edges added to tree
        :param x_init: tuple, initial location
        :param x_goal: tuple, goal location
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        """
        super().__init__(X, Q, x_init, x_goal, max_samples, r, prc)

    def rrt_search(self):

        self.add_vertex(0, self.x_init)
        self.add_edge(0, self.x_init, None)
        x_new=self.x_init
        while True:
            for q in self.Q:  # iterate over different edge lengths until solution found or time out
                for i in range(q[1]):  # iterate over number of edges of given length to add
                    x_new, x_nearest = self.new_and_near(0, x_new)

                    if x_new is None:
                        continue

                    # connect shortest valid edge
                    self.connect_to_point(0, x_nearest, x_new)

                    solution = self.check_solution()
                    if solution[0]:
                        return solution[1]

