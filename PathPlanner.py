# A star (or any other path planning algorithm here)

from Node import Node, FindNodeDist, NODE_MAX_VALUE

import sys
from queue import PriorityQueue
import numpy as np

class PathPlanner:
    def __init__(self, nodes_list):
        self.nodes_list = nodes_list
        self.num_nodes = len(nodes_list)
        self.nodes_f = NODE_MAX_VALUE*np.ones(self.num_nodes)
        self.nodes_cost = NODE_MAX_VALUE*np.ones(self.num_nodes)
        self.nodes_h = -1*np.ones(self.num_nodes)
        self.nodes_prev_node = [Node] * self.num_nodes
        self.current_start_node_id = -1

    def reset_graph(self):
        self.nodes_f = np.ones(self.num_nodes)
        self.nodes_cost = NODE_MAX_VALUE*np.ones(self.num_nodes)
        self.nodes_h = -1*np.ones(self.num_nodes)
        self.nodes_prev_node = [Node] * self.num_nodes
        self.current_start_node_id = -1

    

    def astar(self, start_node: Node, end_node: Node):
        # separate f, g, h variables for larger maps? use matrices, np

        pq = PriorityQueue()
        if self.current_start_node_id != start_node.id:
            self.reset_graph()
            self.current_start_node_id = start_node.id

        if self.nodes_cost[end_node.id] < NODE_MAX_VALUE:
            # path has already been found before
            path = []
            n = end_node
            while n != start_node:
                path.insert(0, n)
                n = self.nodes_prev_node[n.id]
            path.insert(0, n)
            return path, self.nodes_cost[end_node.id]

        self.nodes_cost[start_node.id] = 0
        pq.put(start_node)
        reached_end = False
        while not pq.empty() and (not reached_end):
            curr_node = pq.get()
            adjacentNodes = curr_node.adj_nodes
            for n, cost in adjacentNodes:
                total_cost = self.nodes_cost[curr_node.id] + cost
                if self.nodes_cost[n.id] > total_cost:
                    self.nodes_cost[n.id] = total_cost
                    if self.nodes_h[n.id] == -1:
                        self.nodes_h[n.id] = FindNodeDist(n, end_node)
                    n.f = self.nodes_cost[n.id] + self.nodes_h[n.id]
                    self.nodes_f[n.id] = n.f
                    pq.put(n)
                    self.nodes_prev_node[n.id] = curr_node
            if curr_node == end_node:
                reached_end = True

        path = []
        final_cost = self.nodes_cost[end_node.id]
        if final_cost < NODE_MAX_VALUE:
            # found path
            n = end_node
            while n != start_node:
                path.insert(0, n)
                n = self.nodes_prev_node[n.id]
            path.insert(0, n)
        else:
            #likely because it has not been found before, try working backwards
            print("uh oh")
            pass

        return path, final_cost
    
    # find shortest path between two nodes
    def get_path(self, start_node: Node, end_node: Node):
        if self.current_start_node_id != start_node.id:
            self.explore()

        path = []
        final_cost = self.nodes_cost[end_node.id]
        if final_cost < NODE_MAX_VALUE:
            # found path
            n = end_node
            while n != start_node:
                path.insert(0, n)
                n = self.nodes_prev_node[n.id]
            path.insert(0, n)

        return path, final_cost
    
    # dijkstra to explore every node from the start point
    def explore(self, start_node: Node):
        if self.current_start_node_id != start_node.id:
            self.reset_graph()
            self.current_start_node_id = start_node.id

        pq = PriorityQueue()
        self.nodes_cost[start_node.id] = 0
        start_node.f = 0
        pq.put(start_node)
        while not pq.empty():
            curr_node = pq.get()
            adjacentNodes = curr_node.adj_nodes
            for n, cost in adjacentNodes:
                total_cost = self.nodes_cost[curr_node.id] + cost
                if self.nodes_cost[n.id] > total_cost:
                    self.nodes_cost[n.id] = total_cost
                    n.f = total_cost
                    pq.put(n)
                    self.nodes_prev_node[n.id] = curr_node
        return self.nodes_cost

    def floyd_warshall(self):
        dist = NODE_MAX_VALUE + np.zeros([self.num_nodes, self.num_nodes], dtype=np.float64)
        print(dist)
        for n1 in self.nodes_list:
            i = int(n1.id)
            for n2, cost in n1.adj_nodes:
                j = int(n2.id)
                dist[i][j] = cost
            dist[i][i] = 0
        for k in range(0, self.num_nodes):
            for i in range(0, self.num_nodes):
                for j in range(i, self.num_nodes):
                    c = dist[i][k] + dist[k][j]
                    if dist[i][j] > c:
                        dist[i][j] = c
                        dist[j][i] = c

    def delivery_dist(self, delivery_nodes):
        dist = NODE_MAX_VALUE + np.zeros([len(delivery_nodes), len(delivery_nodes)], dtype=np.float64)
        for i in range(0, len(delivery_nodes) - 1):
            n_from = delivery_nodes[i]
            self.explore(n_from)
            for j in range(i + 1, len(delivery_nodes)):
                n_to = delivery_nodes[j]
                cost = self.nodes_cost[int(n_to.id)]
                dist[i][j] = cost
                dist[j][i] = cost
        return dist

    def delivery_dist_estimate(self, delivery_nodes):
        dist = NODE_MAX_VALUE + np.zeros([len(delivery_nodes), len(delivery_nodes)], dtype=np.float64)
        for i in range(0, len(delivery_nodes) - 1):
            n_from = delivery_nodes[i]
            for j in range(i, len(delivery_nodes)):
                n_to = delivery_nodes[j]
                cost = FindNodeDist(n_from, n_to)
                dist[i][j] = cost
                dist[j][i] = cost
        return dist


