from Node import Node, FindNodeDist
import random
import numpy as np
import matplotlib.pyplot as plt

import time

def AddEdge(node1, node2):
    dist = FindNodeDist(node1, node2)
    node1.adj_nodes.append((node2, dist))
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
                nodes_list.append(Node(str(i), gen_x, gen_y))
    
    # get n nearest nodes for each node
    st = time.time()
    for node in nodes_list:
        current_id = int(node.id)
        nearest_found = 0
        radius = 1
        while nearest_found < num_edges:
            x_range = [max(node.x - radius, x_min), min(node.x + radius, x_max)]
            y_range = [max(node.y - radius, y_min), min(node.y + radius, y_max)]
            for x in range(x_range[0], x_range[1] + 1):
                for y in y_range:
                    other_node_id = int(occupied[y][x])
                    if other_node_id != -1:
                        nearest_found += 1
                        AddEdge(node, nodes_list[other_node_id])
                    if nearest_found >= num_edges:
                        break
                if nearest_found >= num_edges:
                    break
            if nearest_found >= num_edges:
                break
            for y in range(y_range[0] + 1, y_range[1]):
                for x in x_range:
                    other_node_id = int(occupied[y][x])
                    if other_node_id != -1:
                        nearest_found += 1
                        AddEdge(node, nodes_list[other_node_id])
                    if nearest_found >= num_edges:
                        break
                if nearest_found >= num_edges:
                    break
            radius += 1
    et = time.time()
    print(et - st) # 0.02s for 1000 nodes, 0.05s for 5000 nodes, 0.09 for 10000 nodes

    '''
    st = time.time()
    nodes_matrix = np.zeros((num_nodes, num_nodes), dtype=int)
    for i in range(num_nodes):
        for j in range(i+1, num_nodes):
            nodes_matrix[i][j] = FindNodeDist(nodes_list[i], nodes_list[j])
            nodes_matrix[j][i] = nodes_matrix[i][j]

    print(nodes_matrix)

    min_dist_indices = np.argpartition(nodes_matrix, kth=num_edges+1, axis=1)[:, :num_edges+1]
    for i in range(num_nodes):
        for j in min_dist_indices[i]:
            if j != i:
                AddEdge(nodes_list[i], nodes_list[j])
    et = time.time()
    print(et - st) # 0.6s for 1000 nodes, 16.1s for 5000 nodes
    '''

    # TODO: save map to file

    return nodes_list

def PlotMap(nodes_list):
    fig = plt.figure()
    ax = fig.add_subplot(111)

    # plot all points at once
    node_x = [node.x for node in nodes_list]
    node_y = [node.y for node in nodes_list]
    ax.scatter(node_x, node_y, marker='o', color='blue')

    # generate list of lines so plt.plot is 1 call (faster display)
    p1 = []
    p2 = []
    for node in nodes_list:
        current_id = int(node.id)
        for sub_node, dist in node.adj_nodes:
            sub_node_id = int(sub_node.id)
            if (sub_node_id <= current_id):
                n = [n for n, d in sub_node.adj_nodes]
                if node in n:
                    continue
            p1.append([node.x, node.y])
            p2.append([sub_node.x, sub_node.y])
    lines = np.c_[p1, p2]
    line_args = lines.reshape(-1, 2, 2).swapaxes(1, 2).reshape(-1, 2)
    ax.plot(*line_args, c='green')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_title('Generated Map')

    # select start and end:
    points = plt.ginput(2)

    start_node = None
    start_node_dist = 1000000000
    end_node = None
    end_node_dist = 1000000000

    n1 = Node("start", points[0][0], points[0][1])
    n2 = Node("end", points[1][0], points[1][1])
    for node in nodes_list:
        dist = FindNodeDist(n1, node)
        if dist < start_node_dist:
            start_node_dist = dist
            start_node = node
        dist = FindNodeDist(n2, node)
        if dist < end_node_dist:
            end_node_dist = dist
            end_node = node


    return start_node, end_node
