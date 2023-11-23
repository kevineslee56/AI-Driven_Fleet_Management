class Truck:
    def __init__(self, id):
        self.id = id
        self.distance = 0
        self.path = []
        self.delivery_nodes = []

    def update_path(self, path, cost):
        self.path = path
        self.distance = cost

    def add_delivery_nodes(self, delivery_nodes):
        for node in delivery_nodes:
            self.delivery_nodes.append(node)
            node.truck = self

    def clear(self):
        self.distance = 0
        self.path = []
        self.delivery_nodes = []

    def _is_valid_operand(self, other_truck):
        return (hasattr(other_truck, "id"))

    def __eq__(self, other_truck):
        if not self._is_valid_operand(other_truck):
            return NotImplemented
        return self.id == other_truck.id
