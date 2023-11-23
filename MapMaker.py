from Node import Node, FindNodeDist
import random
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import to_rgb
import matplotlib.pyplot as plt
import heapq
from queue import Queue

import time

def AddEdge(node1, node2):
    dist = FindNodeDist(node1, node2)
    if not (node2, dist) in node1.adj_nodes:
        node1.adj_nodes.append((node2, dist))
    if not (node1, dist) in node2.adj_nodes:
        node2.adj_nodes.append((node1, dist))

def GenerateMap(num_nodes, num_edges, x_min, x_max, y_min, y_max):
    # Generate (num_nodes) nodes with unique coordinates
    occupied_coordinates = set()
    nodes_list = []
    for i in range(num_nodes):
        repeated = True
        while repeated:
            x = random.randint(x_min, x_max)
            y = random.randint(y_min, y_max)
            repeated = (x, y) in occupied_coordinates
            if not repeated:
                occupied_coordinates.add((x, y))
                nodes_list.append(Node(i, x, y))
    
    # Connect (num_edges) closest nodes to each node
    for node in nodes_list:
        # Find (num_edges) closest nodes
        closest_nodes = []
        distances = []
        for other_node in nodes_list:
            if (other_node != node):
                distances.append((FindNodeDist(node, other_node), other_node))
        
        # Add edges to those closest nodes
        closest_nodes = heapq.nsmallest(num_edges, distances)
        for dist,closest_node in closest_nodes:
            AddEdge(node, closest_node)

    return nodes_list

def PlotMap(nodes_list):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # plot all points at once
    node_x = [node.x for node in nodes_list]
    node_y = [node.y for node in nodes_list]
    ax.scatter(node_x, node_y, marker='.', color='blue', s=[10 for node in nodes_list], zorder=5)

    # generate list of lines so plt.plot is 1 call (faster display)
    segs = []
    colours = []
    for node in nodes_list:
        current_id = int(node.id)
        for sub_node, dist in node.adj_nodes:
            sub_node_id = int(sub_node.id)
            if (sub_node_id <= current_id):
                n = [n for n, d in sub_node.adj_nodes]
                if node in n:
                    continue
            segs.append(((node.x, node.y),(sub_node.x, sub_node.y)))
    line_collection = LineCollection(segs, linewidths=[0.5], colors=to_rgb('green'), zorder=0)
    ax.add_collection(line_collection)

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('Generated Map')

    # select start and end:
    points = plt.ginput(1)

    start_node = None
    start_node_dist = 1000000000

    n1 = Node("start", points[0][0], points[0][1])
    for node in nodes_list:
        dist = FindNodeDist(n1, node)
        if dist < start_node_dist:
            start_node_dist = dist
            start_node = node

    return start_node, ax
