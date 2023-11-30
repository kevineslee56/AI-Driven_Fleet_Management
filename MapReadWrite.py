# read the files/maps created by MapMaker and create a representation with Node class

from Node import Node
from MapMaker import AddEdge

# file structure:
'''
x_min,x_max,y_min,y_max
start_node_id
num_nodes
node_id,node_x,node_y
........
num rows here = num_nodes
........
num_edges
n1_id,n2_id
........
num rows here = num_edges
........
delivery_id1,delivery_id2......

'''

def read_existing_map(USE_EXISTING_MAP):
    if USE_EXISTING_MAP:
        filename = "saved_map.txt"
        try:
            f = open(filename, 'r')
        except:
            return False, None, None, None, [0, 0, 0, 0]
        else:
            try:
                line = f.readline()
                map_ranges = line.split(",")
                map_ranges = [int(val) for val in map_ranges]

                start_node_id = int(f.readline())

                num_nodes = int(f.readline())
                nodes_list = []
                for i in range(0, num_nodes):
                    line = f.readline()
                    node_info = line.split(",")
                    node_info = [int(val) for val in node_info]
                    n_id = int(node_info[0])
                    n_x = int(node_info[1])
                    n_y = int(node_info[2])
                    nodes_list.append(Node(n_id, n_x, n_y))

                num_edges = int(f.readline())
                for i in range(0, num_edges):
                    line = f.readline()
                    edge_info = line.split(",")
                    edge_info = [int(val) for val in edge_info]
                    n1_id = int(edge_info[0])
                    n2_id = int(edge_info[1])
                    n1 = [n for n in nodes_list if n.id == n1_id][0]
                    n2 = [n for n in nodes_list if n.id == n2_id][0]
                    AddEdge(n1, n2)

                line = f.readline()
                delivery_ids = line.split(",")
                delivery_ids = [int(val) for val in delivery_ids]
                deliveries = [d for d in nodes_list if d.id in delivery_ids]

                start_node = [n for n in nodes_list if n.id == start_node_id][0]
                
                return True, nodes_list, start_node, deliveries, map_ranges
            except:
                return False, None, None, None, [0, 0, 0, 0]
    else:
        return False, None, None, None, [0, 0, 0, 0]
    
def write_existing_map(nodes_list, deliveries, start_node, ranges):

    filename = "saved_map.txt"

    with open(filename, 'w') as f:
        range_line = ','.join([str(val) for val in ranges])
        f.write(range_line)
        f.write('\n')

        start_node_line = str(start_node.id)
        f.write(start_node_line)
        f.write('\n')

        num_nodes_line = str(len(nodes_list))
        f.write(num_nodes_line)
        f.write('\n')
        for node in nodes_list:
            node_line = ','.join([str(node.id), str(node.x), str(node.y)])
            f.write(node_line)
            f.write('\n')
        
        edges = []
        for n1 in nodes_list:
            for n2, cost in n1.adj_nodes:
                edge = [n1.id, n2.id]
                edge2 = [n2.id, n1.id]
                if (edge in edges) or (edge2 in edges):
                    pass
                else:
                    edges.append(edge)
        num_edges_line = str(len(edges))
        f.write(num_edges_line)
        f.write('\n')
        for edge in edges:
            edge_line = ','.join([str(val) for val in edge])
            f.write(edge_line)
            f.write('\n')
        
        delivery_ids_line = ','.join([str(d.id) for d in deliveries])
        f.write(delivery_ids_line)
        f.write('\n')

