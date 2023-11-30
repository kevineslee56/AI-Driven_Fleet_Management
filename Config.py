class Config:
    def __init__(self):

        '''map generation configuration'''
        # whether or not we should try to load an existing map
        self.USE_EXISTING_MAP = True

        # number of nodes/vertices in the graph structure:
        self.num_nodes = 500                # suggested max: 10000, for visibility, 500

        # approximate number of connections (edges) per vertex:
        self.num_neighbours = 4             # suggested for all sized maps: 4-6 (for less dense graphs, use higher numbers)

        # map x and y limits:
        self.x_min = 0                      # keep at 0
        self.x_max = 350                    # suggested max 350 for x_min = 0
        self.y_min = 0                      # keep at 0
        self.y_max = 350                    # suggested max 350 for y_min = 0

        '''problem configuration'''
        # number of deliveries (for random generation option)
        self.num_deliveries = 10            # suggested max 1000, for visibility, 50

        # number of available trucks:
        self.num_trucks = 10                # suggested max 10
        self.truck_capacity = 10000000      # currently not used as a value


        '''simulated annealing solver configuration'''
        # note on weights: should add to 1.0, higher weight means more priority
        # distance weight for cost function:
        self.w_dist = 0.8
        # time weight for cost function:
        self.w_time = 0.2

        # initial simulated annealing temp:
        self.initial_temp = 10000           # suggested 10000
        # stop conditions for simulated annealing
        self.stop_condition_1 = 20000       # suggested 20000
        self.stop_condition_2 = 150000      # suggested 150000
        # number of times to run simulated annealing to find optimal solutions:
        self.max_iter = 1                   # suggested 1, n times more will take n times longer to run
        # max allowable time for 1 iteration of simulated annealing to run (in seconds). in worst case, this is doubled:
        self.max_time_per_iter = 120        # suggested 2 minutes (allowing max 4)


