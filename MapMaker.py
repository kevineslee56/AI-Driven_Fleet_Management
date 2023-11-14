from Node import Node, FindNodeDist
import random
import numpy as np
import matplotlib.pyplot as plt

def AddEdge(node1, node2):
    dist = FindNodeDist(node1, node2)
    node1.adj_nodes.append((node2, dist))
    node2.adj_nodes.append((node1, dist))

def GenerateMap(num_nodes, num_edges, x_min, x_max, y_min, y_max):
    nodes_list = []
    for i in range(num_nodes):
        nodes_list.append(Node(str(i), random.randint(x_min, x_max), random.randint(y_min, y_max)))
    
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

    # TODO: save map to file

    return nodes_list

def PlotMap(nodes_list):
    for node in nodes_list:
        plt.scatter(node.x, node.y, marker='o', color='blue')
        for n,dist in node.adj_nodes:
            plt.plot([node.x, n.x], [node.y, n.y], color='green')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Generated Map')
    plt.show()
