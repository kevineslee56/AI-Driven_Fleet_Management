import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
from Node import Node, FindNodeDist

delivery_points = []

def on_pick(event):
    print('here')
    delivery_points.append(event.x, event.y)

def AddDeliveryPoints(nodes_list, ax, fig):
    # del_points = plt.ginput(1)
    # axcut = plt.axes([0.9, 0.0, 0.1, 0.075])
    # bcut = Button(axcut, 'Done', color='red', hovercolor='green')
    
    ax.set_title('custom picker for line data')
    #line, = ax.plot(rand(100), rand(100), 'o', picker=line_picker)
    fig.canvas.mpl_connect('pick_event', on_pick)

    return delivery_points

def DeliveryPoints(nodes_list):
    node_dist = 1000000000

    del_points = plt.ginput(-1,-1, True, mouse_stop=MouseButton.RIGHT)
    print(del_points)

    for point in del_points:
        n = Node("delivery", point[0], point[1])
        
        for node in nodes_list:
            dist = FindNodeDist(n, node)
            if dist < node_dist:
                node_dist = dist
                n = node
        delivery_points.append(n)
    
    return delivery_points


