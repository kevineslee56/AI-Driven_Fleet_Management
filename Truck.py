class Truck:
    def __init__(self, id, capacity):
        self.id = id
        self.key = str(id)
        self.distance = 0
        self.capacity = capacity
        self.destinations = []
        self.full_path = []
        self.packages = []

    def reset(self):
        self.distance = 0
        self.destinations = []
        self.full_path = []
        self.packages = []

    def __eq__(self, other):
        return self.id == other.id