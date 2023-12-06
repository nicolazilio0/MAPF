import cv2
import numpy as np

# from cbs_mapf.planner import Planner
from scipy.spatial import KDTree

import networkx as nx
import matplotlib.pyplot as plt
import heapq

from scipy.spatial import ConvexHull


import random
import time

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

def is_point_inside_any_circle(point, circles):
    for center, radius in circles:
        if np.linalg.norm(point - center) <= radius:
            return True


def is_point_outside_map(point, polygon):
    if cv2.pointPolygonTest(np.array(polygon, dtype=np.int32), tuple(point), False) >= 0:
            return False
    return True
def divide_image_into_grid(image, rows, cols, polygons,map_border):
    # Get image dimensions
    height, width, _ = image.shape

    # Calculate the size of each grid cell
    cell_height = height // rows
    cell_width = width // cols
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
            if any(is_point_inside_any_circle(corner, polygons) for corner in corners):
                cv2.rectangle(grid_image, (x, y), (x + cell_width, y + cell_height), (0, 0, 0), -1)  # Fill the cell in black
                occupancy_matrix[j, i, 2] = 1  # Mark the cell as occupied
            else:
                if is_point_outside_map((center_x, center_y), map_border):
                    occupancy_matrix[j, i, 2] = 1  # Mark the cell as occupied
                    cv2.rectangle(grid_image, (x, y), (x + cell_width, y + cell_height), (0, 0, 0), -1)  # Fill the cell in black

                if i==0 or j==0 or i==cols-1 or j==rows-1:
                    occupancy_matrix[j, i, 2] = 1  # Mark the cell as occupied
                    cv2.rectangle(grid_image, (x, y), (x + cell_width, y + cell_height), (0, 0, 0), -1)  # Fill the cell in black

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
        return (0 <= i < rows and 0 <= j < cols and map_matrix[i, j] == 0 ) or (0 <= i < rows and 0 <= j < cols and map_matrix[i, j] == 2 ) 

    for i in range(rows):
        for j in range(cols):
            current_node = i * cols + j
            if is_valid(i, j):  # Only proceed if the current node is unoccupied
                # Check and add edges with adjacent unoccupied cells
                for (di, dj) in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ni, nj = i + di, j + dj
                    if is_valid(ni, nj):
                        neighbor_node = ni * cols + nj
                        if map_matrix[i, j] == 1 or map_matrix[ni, nj] == 1:
                            adjacency_matrix[current_node, neighbor_node] = 10
                        else:
                            adjacency_matrix[current_node, neighbor_node] = 1

    return adjacency_matrix



def build_weighted_adjacency_matrix(map_matrix):
    rows, cols = map_matrix.shape
    adjacency_matrix = np.zeros((rows * cols, rows * cols), dtype=int)

    def is_valid(i, j):
        return (0 <= i < rows and 0 <= j < cols and map_matrix[i, j] == 0 ) or (0 <= i < rows and 0 <= j < cols and map_matrix[i, j] == 2 ) 

    for i in range(rows):
        for j in range(cols):
            current_node = i * cols + j
            if is_valid(i, j):  # Only proceed if the current node is unoccupied
                # Check and add edges with adjacent unoccupied cells within a distance of up to 5
                for di in range(-5, 6):
                    for dj in range(-5, 6):
                        ni, nj = i + di, j + dj
                        if 0 <= ni < rows and 0 <= nj < cols and is_valid(ni, nj) and (di**2 + dj**2) <= 25:  # Ensure within bounds and distance <= 5
                            neighbor_node = ni * cols + nj

                            # Check if either of the neighbors is occupied
                            if map_matrix[i, j] == 1 or map_matrix[ni, nj] == 1:
                                adjacency_matrix[current_node, neighbor_node] = 20
                        else:
                             if( 0 <= ni < rows and 0 <= nj < cols and is_valid(ni, nj)):   
                                neighbor_node = ni * cols + nj

                                if((di**2 + dj**2) ==1):
                                    adjacency_matrix[current_node, neighbor_node] = 1

    return adjacency_matrix







def manhattan_distance(node1, node2):
    return abs(node1 - node2)


def euclidean_distance(node1, node2):
    return ((node1 - node2) ** 2) ** 0.5





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
            if len(array1) < len(array2):
                array1.insert(i, array1[i - 1])
            else:
                array2.insert(i, array2[i - 1])

        if array2[i] == array3[i]:
            if len(array2) < len(array3):
                array2.insert(i, array2[i - 1])
            else:
                array3.insert(i, array3[i - 1])

        if array1[i] == array3[i]:
            if len(array1) < len(array3):
                array1.insert(i, array1[i - 1])
            else:
                array3.insert(i, array3[i - 1])

    return array1, array2, array3

def random_border_point(grid_size):
    # Create a list of possible border points
    border_points = []
    border_points.extend([(i, 0) for i in range(grid_size)])      # Left border
    border_points.extend([(i, grid_size-1) for i in range(grid_size) ])   # Right border
    border_points.extend([(0, j) for j in range(grid_size) ])    # Top border
    border_points.extend([(grid_size-1, j) for j in range(grid_size) ])  # Bottom border
    

    # Randomly select one of the border points
    selected_point = random.choice(border_points)

    return selected_point



def random_border_point_in_polygon(polygon_vertices):
    if len(polygon_vertices) < 3:
        raise ValueError("A polygon must have at least three vertices.")

    # Identify edges of the polygon
    edges = [(polygon_vertices[i - 1], polygon_vertices[i]) for i in range(len(polygon_vertices))]

    # Create a list of possible points on the border
    border_points = []

    for edge in edges:
        for t in np.linspace(0, 1, 100):  # Adjust the number of points by changing the third parameter
            x = (1 - t) * edge[0][0] + t * edge[1][0]
            y = (1 - t) * edge[0][1] + t * edge[1][1]
            border_points.append((x, y))

    # Randomly select one of the border points
    if border_points:
        selected_point = random.choice(border_points)
        return selected_point
    else:
        raise ValueError("Failed to find a point on the border of the polygon.")




def square_bounding_box(polygon_vertices):
    min_x, min_y = np.min(polygon_vertices, axis=0)
    max_x, max_y = np.max(polygon_vertices, axis=0)

    # Determine the side length of the square bounding box
    side_length = max(max_x - min_x, max_y - min_y)

    # Calculate the corners of the square bounding box
    square_bbox = np.array([
        [min_x, min_y],
        [min_x + side_length, min_y],
        [min_x + side_length, min_y + side_length],
        [min_x, min_y + side_length]
    ])

    return square_bbox



def minimum_bounding_circle(points):
    def welzl(points, boundary):
        if len(boundary) == 3 or len(points) == 0:
            if len(boundary) == 0:
                return None, 0
            elif len(boundary) == 1:
                return boundary[0], 0
            elif len(boundary) == 2:
                center = 0.5 * (boundary[0] + boundary[1])
                radius = 0.5 * np.linalg.norm(boundary[0] - boundary[1])
                return center, radius
            else:  # len(boundary) == 3
                center, radius = circumscribe(boundary[0], boundary[1], boundary[2])
                return center, radius

        r = welzl(points[1:], boundary)

        if r is not None:
            center_r = r[0]
            if center_r is not None and np.linalg.norm(points[0] - center_r) <= r[1]:
                return r

        return welzl(points[1:], boundary + [points[0]])


    def circumscribe(a, b, c):
        d = 2 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
        ux = ((a[0]**2 + a[1]**2) * (b[1] - c[1]) + (b[0]**2 + b[1]**2) * (c[1] - a[1]) + (c[0]**2 + c[1]**2) * (a[1] - b[1])) / d
        uy = ((a[0]**2 + a[1]**2) * (c[0] - b[0]) + (b[0]**2 + b[1]**2) * (a[0] - c[0]) + (c[0]**2 + c[1]**2) * (b[0] - a[0])) / d
        center = np.array([ux, uy])
        radius = np.linalg.norm(center - a)
        return center, radius+8

    # Shuffle the points randomly to improve the average-case performance
    np.random.shuffle(points)

    center, radius = welzl(points, [])
    return center, radius

def spawn_robots(occupancy_matrix):
    # Find indices where occupancy_matrix[i, j, 2] == 0
    unoccupied_indices = np.argwhere(occupancy_matrix[:, :, 2] == 0)

    # Check if there are at least three unoccupied points
    if len(unoccupied_indices) < 3:
        raise ValueError("Not enough unoccupied points in the occupancy matrix.")

    # Randomly select three distinct indices
    selected_indices = unoccupied_indices[np.random.choice(len(unoccupied_indices), size=3, replace=False)]

    return selected_indices





if __name__ == '__main__':
    # Create an image with a white background


    


    # Define arrays of points for two polygons

    map_border=np.array([[0,0],[0,500],[500,200],[500,0]],np.int32)
    map=square_bounding_box(map_border)
    image=np.ones((map[2][0], map[2][1], 3), dtype=np.uint8) * 255



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
    circles=[]
    for polygon in polygons:
        polygon = np.array(polygon, np.int32)
        circle=minimum_bounding_circle(polygon)
        # circles.append(circle)
        # cv2.circle(image, (int(circle[0][0]), int(circle[0][1])), color=(0, 0, 255), radius=int(circle[1]),thickness=-1)



    # Draw the polygons on the image
    # cv2.polylines(image, [polygon1], isClosed=True, color=(0, 0, 255), thickness=1)  # Red polygon with thickness 2
    # cv2.polylines(image, [polygon2], isClosed=True, color=(0, 255, 0), thickness=1)  # Green polygon with thickness 2

    cv2.imshow("Image with Polygons", image)
    cv2.waitKey(0)  # Wait for a key press

    # Divide the image into a grid

    grid_size=100
    cell_size=image.shape[0]//grid_size
    gridded_image,occupancy_matrix=divide_image_into_grid(image,grid_size,grid_size,circles,map_border)




    robot_1_position=spawn_robots(occupancy_matrix)[0]
    robot_2_position=spawn_robots(occupancy_matrix)[1]
    robot_3_position=spawn_robots(occupancy_matrix)[2]


    robot_1_position=np.array([int(robot_1_position[0]*cell_size),int(robot_1_position[1]*cell_size)])
    robot_2_position=np.array([int(robot_2_position[0]*cell_size),int(robot_2_position[1]*cell_size)])
    robot_3_position=np.array([int(robot_3_position[0]*cell_size),int(robot_3_position[1]*cell_size)])

    print(robot_1_position,robot_2_position,robot_3_position)


    # robot_1_position=robot_1_position+int(cell_size/2)
    # robot_2_position=robot_2_position+int(cell_size/2)
    # robot_3_position=robot_3_position+int(cell_size/2)


    # robot_1_position=np.array([50,100])
    # robot_2_position=np.array([300,200])
    # robot_3_position=np.array([300,100])


    gridded_image=cv2.circle(gridded_image, (int(robot_1_position[0]), int(robot_1_position[1])), color=(0, 255, 255), radius=10,thickness=-1)
    gridded_image=cv2.circle(gridded_image, (int(robot_2_position[0]), int(robot_2_position[1])), color=(0, 255, 255), radius=10,thickness=-1)
    gridded_image=cv2.circle(gridded_image, (int(robot_3_position[0]), int(robot_3_position[1])), color=(0, 255, 255), radius=10,thickness=-1)

    occupancy_matrix[int(robot_1_position[0])//cell_size,int(robot_1_position[1])//cell_size,2]=0
    occupancy_matrix[int(robot_2_position[0])//cell_size,int(robot_2_position[1])//cell_size,2]=0
    occupancy_matrix[int(robot_3_position[0])//cell_size,int(robot_3_position[1])//cell_size,2]=0



    # gate_limits=[(gate_line[0][0],gate_line[0][1]),(gate_line[1][0],gate_line[1][1])]

    # for i in range(gate_line[1][0],gate_line[0][0],cell_size):
    #     for j in range(gate_line[1][1],gate_line[0][1]+cell_size,cell_size):
    #         print(j)
    #         occupancy_matrix[j//cell_size,i//cell_size,2]=2

    # occupancy_matrix[gate_center[1]//cell_size,gate_center[0]//cell_size,2]=2
    cv2.line(gridded_image, (gate_line[0][0],gate_line[0][1]), (gate_line[1][0],gate_line[1][1]), (255, 0, 0), 2)
    cv2.imshow("Image with Polygons", gridded_image)

    cv2.waitKey(0)  # Wait for a key press


    '''
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

    
    cv2.imshow("Image with Polygons", gridded_image)
    cv2.waitKey(0)  # Wait for a key press

    for i in path2:
        gridded_image=cv2.circle(gridded_image, (int(i%grid_size)*cell_size+int(cell_size/2), int(i//grid_size)*cell_size+int(cell_size/2)), color=(255, 255, 0), radius=10,thickness=-1)

    for i in path3:
        gridded_image=cv2.circle(gridded_image, (int(i%grid_size)*cell_size+int(cell_size/2), int(i//grid_size)*cell_size+int(cell_size/2)), color=(12, 110, 125), radius=10,thickness=-1)


    
    cv2.imshow("Image with Polygons", gridded_image)
    
    cv2.waitKey(0)  # Wait for a key press


    max_path_length=max(len(path1),len(path2),len(path3))
    if(len(path1)<max_path_length):
        path1.extend([path1[-1]]*(max_path_length-len(path1)))
    if(len(path2)<max_path_length):
        path2.extend([path2[-1]]*(max_path_length-len(path2)))
    if(len(path3)<max_path_length):
        path3.extend([path3[-1]]*(max_path_length-len(path3)))


    for i in range(max_path_length):
        
        cv2.circle(image,(path1[i]%grid_size*cell_size+int(cell_size/2),path1[i]//grid_size*cell_size+int(cell_size/2)),color=(100, 255, 100), radius=10,thickness=-1)
        cv2.circle(image,(path2[i]%grid_size*cell_size+int(cell_size/2),path2[i]//grid_size*cell_size+int(cell_size/2)),color=(100, 100, 255), radius=10,thickness=-1)
        cv2.circle(image,(path3[i]%grid_size*cell_size+int(cell_size/2),path3[i]//grid_size*cell_size+int(cell_size/2)),color=(255, 100, 100), radius=10,thickness=-1)
        cv2.imshow("Image with Polygons", image)
        time.sleep(0.5)
        cv2.waitKey(1)
    '''

    # test with multiple paths

    print(cell_size,grid_size)
    goal_1= random_border_point_in_polygon(map_border)
    print(goal_1)
    goal_1=np.array([int(goal_1[0]),int(goal_1[1])])
    print(int(goal_1[0]//cell_size),int(goal_1[1]//cell_size))

    if goal_1[1]//cell_size==grid_size:
        occupancy_matrix[int(goal_1[1]//cell_size)-1,int(goal_1[0]//cell_size),2]=2
    if goal_1[0]//cell_size==grid_size:
        occupancy_matrix[int(goal_1[1]//cell_size),int(goal_1[0]//cell_size)-1,2]=2
    else:
        occupancy_matrix[int(goal_1[1]//cell_size),int(goal_1[0]//cell_size),2]=2

    cv2.circle(image,(int(goal_1[0]), int(goal_1[1]) ),color=(255, 0, 0), radius=10,thickness=-1)
    goal_1=int(goal_1[0]//cell_size)+int(goal_1[1]//cell_size)*grid_size


    goal_2= random_border_point_in_polygon(map_border)
    print(goal_2)
    goal_2=np.array([int(goal_2[0]),int(goal_2[1])])


    cv2.circle(image,(int(goal_2[0]), int(goal_2[1]) ),color=(255, 0, 0), radius=10,thickness=-1)
    
    if(goal_2[1]//cell_size==grid_size):
        occupancy_matrix[int(goal_2[1]//cell_size)-1,int(goal_2[0]//cell_size),2]=2
    if(goal_2[0]//cell_size==grid_size):
        occupancy_matrix[int(goal_2[1]//cell_size),int(goal_2[0]//cell_size)-1,2]=2
    else:
        occupancy_matrix[int(goal_2[1]//cell_size),int(goal_2[0]//cell_size),2]=2
    
    goal_2=int(goal_2[0]//cell_size)+int(goal_2[1]//cell_size)*grid_size

    goal_3= random_border_point_in_polygon(map_border)
    print(goal_3)
    goal_3=np.array([int(goal_3[0]),int(goal_3[1])])

    if(goal_3[1]//cell_size==grid_size):
        occupancy_matrix[int(goal_3[1]//cell_size)-1,int(goal_3[0]//cell_size),2]=2
    if(goal_3[0]//cell_size==grid_size):
        occupancy_matrix[int(goal_3[1]//cell_size),int(goal_3[0]//cell_size)-1,2]=2
    else:
        occupancy_matrix[int(goal_3[1]//cell_size),int(goal_3[0]//cell_size),2]=2

    print(int(goal_3[0]//cell_size),int(goal_3[1]//cell_size))
    cv2.circle(image,(int(goal_3[0]), int(goal_3[1]) ),color=(255, 0, 0), radius=10,thickness=-1)
    goal_3=int(goal_3[0]//cell_size)+int(goal_3[1]//cell_size)*grid_size


    print(goal_1,goal_2,goal_3)


    cv2.imshow("Image with Polygons", image)
    cv2.waitKey(0)  # Wait for a key press

    adjacency_matrix = build_adjacency_matrix(occupancy_matrix[:,:,2])

    # Create a graph from the adjacency matrix
    G = nx.from_numpy_array(adjacency_matrix)

    # Draw the graph
    pos = {i: (i // occupancy_matrix.shape[1], i % occupancy_matrix.shape[1]) for i in range(occupancy_matrix.size)}
    nx.draw(G, pos, with_labels=True, font_weight='bold', node_size=500, node_color='skyblue', font_color='black', font_size=10, edge_color='gray', linewidths=1, alpha=0.7)
    # edge_labels = {(edge[0], edge[1]): G.get_edge_data(*edge)['weight'] for edge in G.edges()}
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    # plt.show()



    goals=[goal_1,goal_2,goal_3]
    # Find the A* path
    robots=[robot_1_position,robot_2_position,robot_3_position]
    
    best_paths=[]
    for robot in robots:
        len_path=123549854357345
        best_path_robot=None
        for goal in goals:
            start_node = (int(robot[0])//cell_size)+(int(robot[1])//cell_size)*grid_size
            goal_node = goal
            
            path1 = astar(G, start_node, goal_node, heuristic=euclidean_distance)
            if len(path1)<len_path:
            
                len_path=len(path1)
                best_path_robot=path1
    
        best_paths.append(best_path_robot)

    max_path_length=max(len(best_paths[0]),len(best_paths[1]),len(best_paths[2]))
    if(len(best_paths[0])<max_path_length):
        best_paths[0].extend([best_paths[0][-1]]*(max_path_length-len(best_paths[0])))
    if(len(best_paths[1])<max_path_length):
        best_paths[1].extend([best_paths[1][-1]]*(max_path_length-len(best_paths[1])))
    if(len(best_paths[2])<max_path_length):
        best_paths[2].extend([best_paths[2][-1]]*(max_path_length-len(best_paths[2])))


    for i in range(max_path_length):
        
        cv2.circle(image,(best_paths[0][i]%grid_size*cell_size+int(cell_size/2),best_paths[0][i]//grid_size*cell_size+int(cell_size/2)),color=(100, 255, 100), radius=5,thickness=-1)
        cv2.circle(image,(best_paths[1][i]%grid_size*cell_size+int(cell_size/2),best_paths[1][i]//grid_size*cell_size+int(cell_size/2)),color=(100, 100, 255), radius=5,thickness=-1)
        cv2.circle(image,(best_paths[2][i]%grid_size*cell_size+int(cell_size/2),best_paths[2][i]//grid_size*cell_size+int(cell_size/2)),color=(255, 100, 100), radius=5,thickness=-1)
        cv2.imshow("Image with Polygons", image)
        time.sleep(0.5)
        cv2.waitKey(1)



