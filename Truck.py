class Truck:
    def __init__(self, id, capacity):
        self.id = id
        self.key = str(id)
        self.distance = 0
        self.capacity = capacity
        # used by brute force solver:
        self.delivery_nodes = []
        # used by simulated annealing solver:
        self.packages = []

    # helper functions used by brute force solver:
    def update_path(self, cost, delivery_order):
        self.distance = cost
        self.packages = delivery_order

    def add_delivery_nodes(self, delivery_nodes):
        for node in delivery_nodes:
            self.delivery_nodes.append(node)
            node.truck = self

    # clearing functions for both solvers:
    def clear(self):
        self.reset()

    def reset(self):
        self.distance = 0
        self.delivery_nodes = []
        self.packages = []

    def __eq__(self, other):
        return (self.id == other.id) and (self.capacity == other.capacity)