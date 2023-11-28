import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
from Node import Node, FindNodeDist
import random

delivery_points = []

# User inputted delivery points - select points on the map (will show up as crosshairs) and right click when done
def DeliveryPoints(nodes_list):

    del_points = plt.ginput(-1,-1, True, mouse_stop=MouseButton.RIGHT)
    print(del_points)

    for point in del_points:
        del_node = None
        del_node_dist = 1000000000
        n = Node("delivery", point[0], point[1])

        for node in nodes_list:
            dist = FindNodeDist(n, node)
            if dist < del_node_dist:
                del_node_dist = dist
                del_node = node
        delivery_points.append(del_node)
    
    return delivery_points

# If the user does not select any points (right-click after selecting warehouse point)
def RandomDeliveryPoints(nodes_list, num_nodes):
    num_deliveries = int(num_nodes/10)
    delivery_points = random.sample(nodes_list, num_deliveries)
    return delivery_points


