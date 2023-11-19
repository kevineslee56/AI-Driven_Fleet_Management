import math
from functools import total_ordering

NODE_MAX_VALUE = 1000000000
@total_ordering
class Node:
    def __init__(self, id: int, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.adj_nodes = []
        self.f = NODE_MAX_VALUE
        self.prev_node = None
        self.is_delivery = False

    def _is_valid_operand(self, other_node):
        return (hasattr(other_node, "f"))

    def __lt__(self, other_node):
        if not self._is_valid_operand(other_node):
            return NotImplemented
        return self.f < other_node.f

    def __eq__(self, other_node):
        if not self._is_valid_operand(other_node):
            return NotImplemented
        return self.id == other_node.id

def FindNodeDist(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
