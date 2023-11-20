from Node import Node, FindNodeDist
import random
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import to_rgb
import matplotlib.pyplot as plt

import time

def AddEdge(node1, node2):
    dist = FindNodeDist(node1, node2)
    if not (node2, dist) in node1.adj_nodes:
        node1.adj_nodes.append((node2, dist))
    if not (node1, dist) in node2.adj_nodes:
        node2.adj_nodes.append((node1, dist))

def GenerateMap(num_nodes, num_edges, x_min, x_max, y_min, y_max):
    #num_nodes = int((x_max - x_min) * (y_max - y_min) * 0.2) # set num_nodes based on density desired
   
    # prevent nodes from being generated on the same point
    occupied = -np.ones((y_max - y_min + 1, x_max - x_min + 1))
    nodes_list = []
    for i in range(num_nodes):
        repeated = True
        while repeated:
            gen_x = random.randint(x_min, x_max)
            gen_y = random.randint(y_min, y_max)
            repeated = occupied[gen_y - y_min][gen_x - x_min] != -1
            if not repeated:
                occupied[gen_y - y_min][gen_x - x_min] = i
                nodes_list.append(Node(i, gen_x, gen_y))
    # get n nearest nodes for each node. 
    # TODO: chance of not fully connected graph, fix this
    for node in nodes_list:
        current_id = int(node.id)
        nearest_found = 0
        # loop in spiral
        rel_x = rel_y = 0
        dx = 0
        dy = -1
        while nearest_found < num_edges:
            coord_x = node.x + rel_x
            coord_y = node.y + rel_y
            if (coord_x >= x_min) and (coord_x <= x_max) and (coord_y >= y_min) and (coord_y <= y_max):
                other_node_id = int(occupied[coord_y][coord_x])
                if (other_node_id != -1) and (other_node_id != current_id):
                    nearest_found += 1
                    AddEdge(node, nodes_list[other_node_id])
            if rel_x == rel_y or (rel_x < 0 and rel_x == -rel_y) or (rel_x > 0 and rel_x == 1-rel_y):
                dx, dy = -dy, dx
            rel_x, rel_y = rel_x + dx, rel_y + dy

    # TODO: save map to file

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
