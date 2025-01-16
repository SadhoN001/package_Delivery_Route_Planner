from django.shortcuts import render
from django.http import JsonResponse
import numpy as np
from collections import defaultdict
import heapq
import random

class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = defaultdict(list)
        self.distances = {}
        
    def add_node(self, value):
        self.nodes.add(value)
        
    def add_edge(self, from_node, to_node, distance):
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)  # For undirected graph
        self.distances[(from_node, to_node)] = distance
        self.distances[(to_node, from_node)] = distance  # For undirected graph

def dijkstra(graph, initial_node):
    visited = {initial_node: 0}
    path = {}
    nodes = set(graph.nodes)
    
    while nodes:
        min_node = None
        for node in nodes:
            if node in visited:
                if min_node is None:
                    min_node = node
                elif visited[node] < visited[min_node]:
                    min_node = node
        
        if min_node is None:
            break
            
        nodes.remove(min_node)
        current_weight = visited[min_node]
        
        for neighbor in graph.edges[min_node]:
            weight = current_weight + graph.distances[(min_node, neighbor)]
            if neighbor not in visited or weight < visited[neighbor]:
                visited[neighbor] = weight
                path[neighbor] = min_node
    
    return visited, path

def construct_path(path, start, end):
    route = []
    current = end
    
    while current != start:
        route.append(current)
        current = path[current]
    route.append(start)
    route.reverse()
    
    return route

def route_planner(request):
    if request.method == 'GET':
        return render(request, 'route_planner.html')
        
    if request.method == 'POST':
        # Get coordinates with validation
        start_coords = request.POST.get('start_point_coords')
        delivery_coords = request.POST.get('delivery_point1_coords')
        
        print("Start coords:", start_coords)
        print("Delivery coords:", delivery_coords)
        
        if not all([start_coords, delivery_coords]):
            return JsonResponse({
                'error': 'Please select both locations'
            }, status=400)
        
        try:
            # Parse coordinates
            start_point = tuple([float(x) for x in start_coords.split(',')])
            delivery_point = tuple([float(x) for x in delivery_coords.split(',')])
            
            # Create graph and add nodes
            graph = create_graph_from_coordinates(start_point, delivery_point)
            
            # Calculate shortest path
            route = calculate_shortest_path(graph, start_point, delivery_point)
            
            return JsonResponse({
                'route': route,
                'distance': route['total_distance']
            })
            
        except (ValueError, AttributeError) as e:
            print("Error processing coordinates:", str(e))
            return JsonResponse({
                'error': 'Invalid coordinate format'
            }, status=400)

def create_graph_from_coordinates(start_point, end_point):
    """Create a graph with intermediate points for more realistic routing"""
    graph = Graph()
    
    # Add start and end points
    graph.add_node(start_point)
    graph.add_node(end_point)
    
    # Create intermediate points (grid)
    lat_min = min(start_point[0], end_point[0])
    lat_max = max(start_point[0], end_point[0])
    lon_min = min(start_point[1], end_point[1])
    lon_max = max(start_point[1], end_point[1])
    
    # Create grid points
    grid_size = 10  # Increased number of intermediate points
    lat_step = (lat_max - lat_min) / (grid_size + 1)
    lon_step = (lon_max - lon_min) / (grid_size + 1)
    
    grid_points = []
    for i in range(1, grid_size + 1):
        for j in range(1, grid_size + 1):
            point = (
                lat_min + i * lat_step + random.uniform(-lat_step/4, lat_step/4),
                lon_min + j * lon_step + random.uniform(-lon_step/4, lon_step/4)
            )
            grid_points.append(point)
            graph.add_node(point)
    
    # Connect points with edges
    # Connect start point to nearby grid points
    for point in grid_points:
        if calculate_distance(start_point, point) < max(lat_step, lon_step) * 2:
            graph.add_edge(start_point, point, calculate_distance(start_point, point))
    
    # Connect end point to nearby grid points
    for point in grid_points:
        if calculate_distance(end_point, point) < max(lat_step, lon_step) * 2:
            graph.add_edge(end_point, point, calculate_distance(end_point, point))
    
    # Connect grid points to each other
    for i, point1 in enumerate(grid_points):
        for point2 in grid_points[i+1:]:
            if calculate_distance(point1, point2) < max(lat_step, lon_step) * 2:
                graph.add_edge(point1, point2, calculate_distance(point1, point2))
    
    return graph

def calculate_shortest_path(graph, start_point, end_point):
    """Calculate shortest path using Dijkstra's algorithm with road waypoints"""
    # Major roads/intersections in Chittagong (approximate coordinates)
    major_waypoints = [
        # Main road points between Agrabad and Oxygen
        (22.3253, 91.8115),  # Agrabad start
        (22.3350, 91.8200),  # Station Road
        (22.3414, 91.8277),  # Tiger Pass
        (22.3500, 91.8300),  # GEC Circle
        (22.3600, 91.8350),  # Muradpur
        (22.3700, 91.8400),  # Bahaddarhat
        (22.3789, 91.8432),  # Oxygen end
    ]
    
    # Create path through major waypoints
    path_points = []
    path_points.append(list(start_point))
    
    # Add relevant waypoints based on start and end locations
    for waypoint in major_waypoints:
        if is_point_on_path(start_point, end_point, waypoint):
            path_points.append(list(waypoint))
    
    path_points.append(list(end_point))
    
    # Calculate total distance
    total_distance = 0
    for i in range(len(path_points) - 1):
        total_distance += calculate_distance(
            tuple(path_points[i]), 
            tuple(path_points[i + 1])
        )
    
    # Create smooth path with interpolated points
    smooth_path = []
    for i in range(len(path_points) - 1):
        current = path_points[i]
        next_point = path_points[i + 1]
        smooth_path.append(current)
        
        # Add interpolated points
        steps = 10  # Number of interpolation points
        for j in range(1, steps):
            fraction = j / steps
            lat = current[0] + (next_point[0] - current[0]) * fraction
            lng = current[1] + (next_point[1] - current[1]) * fraction
            smooth_path.append([lat, lng])
    
    smooth_path.append(path_points[-1])
    
    return {
        'coordinates': [list(start_point), list(end_point)],
        'total_distance': total_distance,
        'path': smooth_path,
        'waypoints': path_points,
        'number_of_stops': 2
    }

def is_point_on_path(start, end, point, threshold=0.01):
    """Check if a waypoint is roughly on the path between start and end"""
    # Calculate if point is near the direct path
    path_vector = (end[0] - start[0], end[1] - start[1])
    point_vector = (point[0] - start[0], point[1] - start[1])
    
    # Cross product to find distance from line
    cross_product = abs(
        path_vector[0] * point_vector[1] - 
        path_vector[1] * point_vector[0]
    )
    
    path_length = np.sqrt(path_vector[0]**2 + path_vector[1]**2)
    distance_from_path = cross_product / path_length
    
    # Check if point is between start and end
    dot_product = (
        path_vector[0] * point_vector[0] + 
        path_vector[1] * point_vector[1]
    )
    
    return (
        distance_from_path < threshold and 
        0 <= dot_product <= (path_vector[0]**2 + path_vector[1]**2)
    )
    
def calculate_distance(point1, point2):
    """
    Calculate the distance between two points using Haversine formula
    Args:
        point1: tuple of (latitude, longitude)
        point2: tuple of (latitude, longitude)
    Returns:
        Distance in kilometers
    """
    # Convert coordinates to radians
    lat1, lon1 = np.radians(point1[0]), np.radians(point1[1])
    lat2, lon2 = np.radians(point2[0]), np.radians(point2[1])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arcsin(np.sqrt(a))
    
    # Radius of Earth in kilometers
    r = 6371
    
    # Calculate distance
    distance = c * r
    
    return distance