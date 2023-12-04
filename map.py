import cv2
import numpy as np

# from cbs_mapf.planner import Planner
from scipy.spatial import KDTree

import networkx as nx
import matplotlib.pyplot as plt
import heapq

import random


def divide_image_into_grid(image, rows, cols):
    # Get image dimensions
    height, width, _ = image.shape

    # Calculate the size of each grid cell
    cell_height = height // rows
    cell_width = width // cols

    # Create an empty image to draw the grid
    grid_image = image.copy()

    # Draw vertical lines
    for i in range(1, cols):
        x = i * cell_width
        cv2.line(grid_image, (x, 0), (x, height), (255, 0, 0), 1)

    # Draw horizontal lines
    for j in range(1, rows):
        y = j * cell_height
        cv2.line(grid_image, (0, y), (width, y), (255, 0, 0), 1)

    return grid_image


def is_point_inside_any_polygon(point, polygons):
    for polygon in polygons:
        if cv2.pointPolygonTest(np.array(polygon, dtype=np.int32), tuple(point), False) >= 0:
            return True
    return False
def divide_image_into_grid(image, rows, cols, polygons):
    # Get image dimensions
    height, width, _ = image.shape

    # Calculate the size of each grid cell
    cell_height = height // rows
    cell_width = width // cols
    print(cell_height,cell_width )
    # Create an empty image to draw the grid
    grid_image = image.copy()

    # Initialize the occupancy matrix
    occupancy_matrix = np.zeros((rows, cols, 3), dtype=np.uint8)

    # Iterate through cells and mark those intersecting with polygons as black
    for i in range(cols):
        for j in range(rows):
            x = i * cell_width
            y = j * cell_height

            # Calculate the center coordinates of the cell
            center_x = x + cell_width / 2
            center_y = y + cell_height / 2
            cv2.circle(grid_image, (int(center_x), int(center_y)), color=(0, 0, 255), radius=2)
            # Check each corner of the cell
            corners = [(x, y), (x + cell_width, y), (x, y + cell_height), (x + cell_width, y + cell_height)]
            occupancy_matrix[j, i, 0] = center_y  # Store y-coordinate
            occupancy_matrix[j, i, 1] = center_x  # Store x-coordinate
            # Check if any corner of the cell is inside any polygon
            if any(is_point_inside_any_polygon(corner, polygons) for corner in corners):
                cv2.rectangle(grid_image, (x, y), (x + cell_width, y + cell_height), (0, 0, 0), -1)  # Fill the cell in black
                occupancy_matrix[j, i, 2] = 1  # Mark the cell as occupied
            else:
                # Draw vertical lines
                if i < cols - 1:
                    cv2.line(grid_image, (x + cell_width, y), (x + cell_width, y + cell_height), (255, 0, 0), 1)

                # Draw horizontal lines
                if j < rows - 1:
                    cv2.line(grid_image, (x, y + cell_height), (x + cell_width, y + cell_height), (255, 0, 0), 1)

    return grid_image, occupancy_matrix


def vertices_to_obsts(obsts):
    def drawRect(v0, v1):
        o = []
        base = abs(v0[0] - v1[0])
        side = abs(v0[1] - v1[1])
        for xx in range(0, base, 30):
            o.append((v0[0] + xx, v0[1]))
            o.append((v0[0] + xx, v0[1] + side ))
        o.append((v0[0] + base, v0[1]))
        o.append((v0[0] + base, v0[1] + side ))
        for yy in range(0, side, 30):
            o.append((v0[0], v0[1] + yy))
            o.append((v0[0] + base , v0[1] + yy))
        o.append((v0[0], v0[1] + side))
        o.append((v0[0] + base , v0[1] + side))
        return o
        
    static_obstacles = []
    for vs in obsts.values():
        static_obstacles.extend(drawRect(vs[0], vs[1]))
    return static_obstacles



def build_adjacency_matrix(map_matrix):
    rows, cols = map_matrix.shape
    adjacency_matrix = np.zeros((rows * cols, rows * cols), dtype=int)

    def is_valid(i, j):
     
        return 0 <= i < rows and 0 <= j < cols and map_matrix[i, j] == 0

    for i in range(rows):
        for j in range(cols):
            current_node = i * cols + j
            if is_valid(i, j):  # Only proceed if the current node is unoccupied
                # Check and add edges with adjacent unoccupied cells
                for (di, dj) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ni, nj = i + di, j + dj
                    if is_valid(ni, nj):
                        neighbor_node = ni * cols + nj
                        adjacency_matrix[current_node, neighbor_node] = 1

    return adjacency_matrix

def euclidean_distance(node1, node2):
    return abs(node1 - node2)



def astar(graph, start, goal, heuristic):
    # Priority queue for A* algorithm
    open_set = [(0, start)]
    closed_set = set()
    g_values = {node: float('inf') for node in graph.nodes}
    g_values[start] = 0
    came_from = {}  # Store the parent node for each visited node

    while open_set:
        _, current_node = heapq.heappop(open_set)

        if current_node == goal:
            path = [goal]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            path.reverse()
            return path

        closed_set.add(current_node)

        for neighbor in graph.neighbors(current_node):
            if neighbor in closed_set:
                continue

            tentative_g = g_values[current_node] + graph[current_node][neighbor].get('weight', 1)

            if tentative_g < g_values[neighbor]:
                g_values[neighbor] = tentative_g
                f_value = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_value, neighbor))
                came_from[neighbor] = current_node  # Update the parent node

    return None  # No path found

def generate_random_polygon(image_size,number_of_polygons):
    # Random number of edges between 3 and 8
    polygons=[]
    max_polygon_size=(30,30)
    for _ in range(number_of_polygons):
        num_edges = random.randint(3, 8)
        vertices = []

    # Generate random vertices
        start_point=(random.randint(0, image_size[1] - 1),random.randint(0, image_size[0] - 1))
        angles = sorted([random.uniform(0, 2*np.pi) for _ in range(num_edges)])

        # Generate vertices based on the starting point and angles
        vertices = [start_point]
        for angle in angles:
            x = int(start_point[0] + max_polygon_size[1] * np.cos(angle))
            y = int(start_point[1] + max_polygon_size[0] * np.sin(angle))
            vertices.append((x, y))
        polygons.append(vertices)
    return polygons




def check_conflicts(array1, array2, array3):
    # Iterate through the arrays pairwise
    min_length = min(len(array1), len(array2), len(array3))
    for i in range(min_length - 1, 0, -1):
        if array1[i] == array2[i]:
            array1.insert(i, array1[i - 1])
            array2.insert(i, array2[i - 1])

        if array2[i] == array3[i]:
            array2.insert(i, array2[i - 1])
            array3.insert(i, array3[i - 1])

        if array1[i] == array3[i]:
            array1.insert(i, array1[i - 1])
            array3.insert(i, array3[i - 1])

    return array1, array2, array3




if __name__ == '__main__':
    # Create an image with a white background
    image = np.ones((500, 500, 3), dtype=np.uint8) * 255

    # Define arrays of points for two polygons

    map_border=np.array([[0,0],[0,500],[500,500],[500,0]],np.int32)

    cv2.polylines(image, [map_border], isClosed=True, color=(0, 0, 0), thickness=4)  # Red polygon with thickness 2


    gate_center=[200,0]
    gate_orientation= 1 # 1 for vertical, 0 for horizontal

    if gate_orientation:
        gate_line=np.array([[gate_center[0]+25,gate_center[1]],[gate_center[0]-25,gate_center[1]]],np.int32)

    else:
        gate_line=np.array([[gate_center[0],gate_center[1]+25],[gate_center[0],gate_center[1]-25]],np.int32)








    cv2.polylines(image, [gate_line], isClosed=True, color=(255, 0, 0), thickness=5)  # Red polygon with thickness 2

    #polygon1 = np.array([[100, 100], [200, 75], [200, 100]], np.int32)
    #polygon2 = np.array([[50, 300], [200, 250], [350, 300]], np.int32)

    # Reshape the arrays for compatibility with OpenCV
    # polygon1 = polygon1.reshape((-1, 1, 2))
    # polygon2 = polygon2.reshape((-1, 1, 2))

    # polygons=[polygon1,polygon2]

    # plan_poly=[(100, 100), (50, 200), (100, 300),(300, 50), (200, 200), (300, 400)]


    polygons=generate_random_polygon((500,500),10)

    for polygon in polygons:
        polygon = np.array(polygon, np.int32)
        polygon = polygon.reshape((-1, 1, 2))
        cv2.polylines(image, [polygon], isClosed=True, color=(0, 0, 255), thickness=1)  # Red polygon with thickness 2


    # Draw the polygons on the image
    # cv2.polylines(image, [polygon1], isClosed=True, color=(0, 0, 255), thickness=1)  # Red polygon with thickness 2
    # cv2.polylines(image, [polygon2], isClosed=True, color=(0, 255, 0), thickness=1)  # Green polygon with thickness 2

    cv2.imshow("Image with Polygons", image)
    cv2.waitKey(0)  # Wait for a key press

    # Divide the image into a grid

    grid_size=50
    cell_size=image.shape[0]//grid_size
    gridded_image,occupancy_matrix=divide_image_into_grid(image,grid_size,grid_size,polygons)


    robot_1_position=np.random.uniform(low=0, high=grid_size, size=2)
    robot_2_position=np.random.uniform(low=0, high=grid_size, size=2)
    robot_3_position=np.random.uniform(low=0, high=grid_size, size=2)


    robot_1_position=np.array([int(robot_1_position[0]*cell_size),int(robot_1_position[1]*cell_size)])
    robot_2_position=np.array([int(robot_2_position[0]*cell_size),int(robot_2_position[1]*cell_size)])
    robot_3_position=np.array([int(robot_3_position[0]*cell_size),int(robot_3_position[1]*cell_size)])

    print(robot_1_position,robot_2_position,robot_3_position)


    robot_1_position=robot_1_position+int(cell_size/2)
    robot_2_position=robot_2_position+int(cell_size/2)
    robot_3_position=robot_3_position+int(cell_size/2)


    # robot_1_position=np.array([50,100])
    # robot_2_position=np.array([300,200])
    # robot_3_position=np.array([300,100])


    gridded_image=cv2.circle(gridded_image, (int(robot_1_position[0]), int(robot_1_position[1])), color=(0, 255, 255), radius=10,thickness=-1)
    gridded_image=cv2.circle(gridded_image, (int(robot_2_position[0]), int(robot_2_position[1])), color=(0, 255, 255), radius=10,thickness=-1)
    gridded_image=cv2.circle(gridded_image, (int(robot_3_position[0]), int(robot_3_position[1])), color=(0, 255, 255), radius=10,thickness=-1)

    occupancy_matrix[int(robot_1_position[0])//cell_size,int(robot_1_position[1])//cell_size,2]=0
    occupancy_matrix[int(robot_2_position[0])//cell_size,int(robot_2_position[1])//cell_size,2]=0
    occupancy_matrix[int(robot_3_position[0])//cell_size,int(robot_3_position[1])//cell_size,2]=0

    occupancy_matrix[int(gate_center[0])//cell_size,int(gate_center[1])//cell_size,2]=0
    

    cv2.imshow("Image with Polygons", gridded_image)





    adjacency_matrix = build_adjacency_matrix(occupancy_matrix[:,:,2])

    # Create a graph from the adjacency matrix
    G = nx.from_numpy_array(adjacency_matrix)

    # Draw the graph
    pos = {i: (i // occupancy_matrix.shape[1], i % occupancy_matrix.shape[1]) for i in range(occupancy_matrix.size)}
    nx.draw(G, pos, with_labels=True, font_weight='bold', node_size=500, node_color='skyblue', font_color='black', font_size=10, edge_color='gray', linewidths=1, alpha=0.7)

    # Display the graph





    # Find the A* path
    start_node = (int(robot_1_position[0])//cell_size)+(int(robot_1_position[1])//cell_size)*grid_size
    goal_node = (int(gate_center[0])//cell_size)+(int(gate_center[1])//cell_size)*grid_size
    path1 = astar(G, start_node, goal_node, heuristic=euclidean_distance)
    
 


    # print("A* robot 1 Path:", path)

    start_node = (int(robot_2_position[0])//cell_size)+(int(robot_2_position[1])//cell_size)*grid_size
    goal_node = (int(gate_center[0])//cell_size)+(int(gate_center[1])//cell_size)*grid_size
    path2 = astar(G, start_node, goal_node, heuristic=euclidean_distance)

    # print("A* robot 2 Path:", path)


    start_node = (int(robot_3_position[0])//cell_size)+(int(robot_3_position[1])//cell_size)*grid_size
    goal_node = (int(gate_center[0])//cell_size)+(int(gate_center[1])//cell_size)*grid_size
    path3 = astar(G, start_node, goal_node, heuristic=euclidean_distance)

    # print("A* robot 2 Path:", path)
    # plt.show()


    path1, path2, path3 = check_conflicts(path1, path2, path3)

    for i in path1:
        gridded_image=cv2.circle(gridded_image, (int(i%grid_size)*cell_size+int(cell_size/2), int(i//grid_size)*cell_size+int(cell_size/2)), color=(0, 255, 0), radius=10,thickness=-1)


    for i in path2:
        gridded_image=cv2.circle(gridded_image, (int(i%grid_size)*cell_size+int(cell_size/2), int(i//grid_size)*cell_size+int(cell_size/2)), color=(255, 255, 0), radius=10,thickness=-1)

    for i in path3:
        gridded_image=cv2.circle(gridded_image, (int(i%grid_size)*cell_size+int(cell_size/2), int(i//grid_size)*cell_size+int(cell_size/2)), color=(12, 110, 125), radius=10,thickness=-1)

    
    
    cv2.imshow("Image with Polygons", gridded_image)
    
    cv2.waitKey(0)  # Wait for a key press

    '''
    plan_poly=[(0,0)]
    plan_poly.append((500,500))
    plan_poly.append((0,500))
    plan_poly.append((500,0))

    print(vertices_to_obsts({1: [(x1,y1), (x1+w1,y1+h1)]}))

    # plan_poly.extend(vertices_to_obsts({1: [(x1,y1), (x1+w1,y1+h1)]}))
    # plan_poly.extend(vertices_to_obsts({1: [(x2,y2), (x2+w2,y2+h2)]}))
    


    for i in range(25):
          for j in range(25) :
              if(occupancy_matrix[j,i,2]==1):

                obstacles=vertices_to_obsts({1: [(i*20,j*20), (i*20+20,j*20+20)]})
                print("i,j", i,j,"obst",obstacles)
                plan_poly.extend(obstacles)
    print(plan_poly)
    cv2.circle(gridded_image, (int(gate_center[0]), int(gate_center[1])), color=(255, 0, 0), radius=10,thickness=-1)



    # Display the image with polygons
    cv2.imshow("Image with Polygons", gridded_image)
    cv2.waitKey(0)  # Wait for a key press
    cv2.imshow("output_image_with_polygons.jpg", image)
    cv2.waitKey(0)  # Wait for a key press
    print("Image with polygons saved successfully!")


    planner=Planner(25, 5, plan_poly)


    dest_robot_1=(gate_center[1]-robot_1_position[0],gate_center[0]-robot_1_position[1])



    print("Start planning")
    result=planner.plan(starts=[(int(robot_1_position[0]), int(robot_1_position[1])),(int(robot_2_position[0]), int(robot_2_position[1])),(int(robot_3_position[0]), int(robot_3_position[1]))],
        goals=[(int(gate_center[0]), int(gate_center[1])),(int(gate_center[0]), int(gate_center[1])),(int(gate_center[0]), int(gate_center[1]))],
        max_iter=300,
        low_level_max_iter=300,
        max_process=2,
        debug=True,
    )





    image=cv2.circle(image, (int(robot_1_position[0]), int(robot_1_position[1])), color=(0, 255, 255), radius=10,thickness=-1)
    image=cv2.circle(image, (int(robot_2_position[0]), int(robot_2_position[1])), color=(0, 255, 255), radius=10,thickness=-1)
    image=cv2.circle(image, (int(robot_3_position[0]), int(robot_3_position[1])), color=(0, 255, 255), radius=10,thickness=-1)

    image=cv2.circle(image, (250, 250), color=(255, 0, 0), radius=10,thickness=-1)
    movements=[]


    # for i in range(len(result[0])):
    #     if i==0:
    #         movements.append((result[0][i][1]-(int(robot_1_position[1])),result[0][i][0]-(int(robot_1_position[0]))))
    #     else:
    #         movements.append((result[0][i][1]-result[0][i-1][1],result[0][i][0]-result[0][i-1][0]))
    # movements=[(mov[0],mov[1]) for mov in movements]
    # print(movements)

    # for i in range(len(movements)):
    #     robot_1_position[0]+=movements[i][0]
    #     robot_1_position[1]+=movements[i][1]
    #     image=cv2.circle(image, (int(robot_1_position[0]), int(robot_1_position[1])), color=(0, 255, 255), radius=10,thickness=-1)

    cv2.polylines(image, [result[0]], isClosed=False, color=(0, 0, 255), thickness=2)  # Red polygon with thickness 2
    cv2.polylines(image, [result[1]], isClosed=False, color=(255, 0, 0), thickness=2)  # Red polygon with thickness 2
    cv2.polylines(image, [result[2]], isClosed=False, color=(0, 255, 0), thickness=2)  # Red polygon with thickness 2




    cv2.imshow("Image with Polygons", image)
    cv2.waitKey(0)  # Wait for a key press


    # Save the image with polygons

    cv2.destroyAllWindows()
'''
