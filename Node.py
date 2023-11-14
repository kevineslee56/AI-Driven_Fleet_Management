import math
import sys
from functools import total_ordering

@total_ordering
class Node:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.adj_nodes = {}
        self.f = sys.maxsize
        self.g = 0
        self.h = 0
        self.prev_node = None

    def updateH(self, other_node):
        self.h = FindNodeDist(self, other_node)

    def _is_valid_operand(self, other_node):
        return (hasattr(other_node, "f"))

    def __lt__(self, other_node):
        if not self._is_valid_operand(other_node):
            return NotImplemented
        return (self.f, self.f < other_node.f, other_node.f)

    def __eq__(self, other_node):
        if not self._is_valid_operand(other_node):
            return NotImplemented
        return (self.id, self.id == other_node.id, other_node.id)

def FindNodeDist(node1, node2):
    return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
