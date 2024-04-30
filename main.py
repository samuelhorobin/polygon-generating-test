import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # Import Poly3DCollection explicitly
import numpy as np

# np.random.seed(3)

class Node:
    def __init__(self, x: int, y: int, z: int):
        self.position = np.array([x, y, z])
        self.neighbours = []

def generate_nodes(n):
    return [Node(np.random.rand(), np.random.rand(), np.random.rand()) for _ in range(n)]

def get_closest_node(subject_node, nodes):
    closest_node = None
    closest_distance = float('inf')

    # print('')
    # print(f'subject_node: {subject_node}')
    # print(f'nodes: {nodes}')
    # print(f'subject_node.neighbours: {subject_node.neighbours}')
    
    for node in nodes:
        if node is subject_node:
            pass
        else:
            distance = np.linalg.norm(subject_node.position - node.position)
            if distance < closest_distance:
                closest_node = node
                closest_distance = distance
    return closest_node

def propagate_node_edges(nodes):
    for node in nodes:
        open_nodes = nodes.copy()
        open_nodes.remove(node)
        while len(node.neighbours) != 3 and open_nodes: 
            closest_node = get_closest_node(subject_node=node, nodes=open_nodes)
            open_nodes.remove(closest_node)

            if node in closest_node.neighbours or len(closest_node.neighbours) == 3:
                pass
            else:
                node.neighbours.append(closest_node)
                
    return nodes

def plot_nodes(nodes):
    # Create a new matplotlib figure and 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Extract coordinates from nodes
    x = [node.position[0] for node in nodes]
    y = [node.position[1] for node in nodes]
    z = [node.position[2] for node in nodes]
    
    # Plot the points
    # ax.scatter(x, y, z, c='blue', marker='o')

    # Plot the edges
    # for node in nodes:
    #     for neighbor in node.neighbours:
    #         ax.plot([node.position[0], neighbor.position[0]],
    #                 [node.position[1], neighbor.position[1]],
    #                 [node.position[2], neighbor.position[2]],
    #                 c='black')
            
    # Plot filled polygons
    for node in nodes:
        for i in range(len(node.neighbours)):
            neighbor1 = node.neighbours[i]
            neighbor2 = node.neighbours[(i + 1) % len(node.neighbours)]
            verts = [node.position, neighbor1.position, neighbor2.position, node.position]
            ax.add_collection3d(Poly3DCollection([verts], facecolors='green', alpha=0.2, linewidths=0))
    
    # Set the labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'{len(nodes)}-pointed polygon')
    
    # Set the limits for the axes
    ax.set_xlim([0, 1])
    ax.set_ylim([0, 1])
    ax.set_zlim([0, 1])
    
    # Show the plot
    plt.show()

n = 12
nodes = generate_nodes(n)
nodes = propagate_node_edges(nodes)
plot_nodes(nodes)
