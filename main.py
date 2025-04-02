import sys
import math
import xml.etree.ElementTree as ET
import tkinter as tk
from tkinter import Canvas, Label, Button, Frame, Scale
import heapq

class Node:
    def __init__(self, id, lat, lon):
        self.id = id
        self.lat = float(lat)
        self.lon = float(lon)
        self.adjacent = {}  # {node_id: distance}
        
    def __str__(self):
        return f"Node({self.id}, {self.lat}, {self.lon})"
        
    def add_neighbor(self, neighbor_id, weight=1):
        self.adjacent[neighbor_id] = weight
        
    def get_connections(self):
        return self.adjacent.keys()
        
    def get_weight(self, neighbor_id):
        return self.adjacent[neighbor_id]
        
    def get_id(self):
        return self.id

class Way:
    def __init__(self, id):
        self.id = id
        self.nodes = []  # List of node IDs
    
    def add_node(self, node_id):
        self.nodes.append(node_id)

class Graph:
    def __init__(self):
        self.nodes = {}  # {node_id: Node}
        self.ways = {}   # {way_id: Way}
        self.node_coords = {}  # {node_id: (lat, lon)}
        self.min_lat = float('inf')
        self.max_lat = float('-inf')
        self.min_lon = float('inf')
        self.max_lon = float('-inf')
    
    def add_node(self, node):
        self.nodes[node.id] = node
        self.node_coords[node.id] = (node.lat, node.lon)
        
        # Update bounding box
        self.min_lat = min(self.min_lat, node.lat)
        self.max_lat = max(self.max_lat, node.lat)
        self.min_lon = min(self.min_lon, node.lon)
        self.max_lon = max(self.max_lon, node.lon)
        
    def add_way(self, way):
        self.ways[way.id] = way
        
    def get_node(self, node_id):
        return self.nodes.get(node_id, None)
        
    def get_way(self, way_id):
        return self.ways.get(way_id, None)
        
    def add_edge(self, src_id, dst_id, weight=1):
        if src_id not in self.nodes:
            print(f"Warning: Source node {src_id} not in graph")
            return
        if dst_id not in self.nodes:
            print(f"Warning: Destination node {dst_id} not in graph")
            return
            
        self.nodes[src_id].add_neighbor(dst_id, weight)
        
    def get_nodes(self):
        return self.nodes.keys()

def haversine(lat1, lon1, lat2, lon2):
    """Calculate the great circle distance between two points on earth"""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # Radius of Earth in kilometers
    return c * r

def point_to_line_distance(px, py, x1, y1, x2, y2):
    """Calculate the shortest distance from point (px,py) to line segment (x1,y1)-(x2,y2)"""
    # Line length
    line_length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    if line_length == 0:  # Line is actually a point
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2), (x1, y1)
    
    # Calculate the projection of point onto the line
    t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (line_length ** 2)))
    
    # Calculate the closest point on the line
    projection_x = x1 + t * (x2 - x1)
    projection_y = y1 + t * (y2 - y1)
    
    # Return the distance from point to projection
    return math.sqrt((px - projection_x) ** 2 + (py - projection_y) ** 2), (projection_x, projection_y)

def parse_osm(file_path):
    print(f"Parsing OSM file: {file_path}")
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
    except Exception as e:
        print(f"Error parsing OSM file: {e}")
        return None
        
    graph = Graph()
    
    # First pass: add all nodes
    for node in root.findall('./node'):
        node_id = node.get('id')
        lat = float(node.get('lat'))
        lon = float(node.get('lon'))
        graph.add_node(Node(node_id, lat, lon))
    
    # Second pass: add all ways (edges)
    for way in root.findall('./way'):
        way_id = way.get('id')
        new_way = Way(way_id)
        
        # Check if this way is for roads/highways (we're only interested in walkable/drivable paths)
        is_highway = False
        for tag in way.findall('./tag'):
            if tag.get('k') == 'highway':
                is_highway = True
                break
                
        if not is_highway:
            continue
        
        # Get all nodes in this way
        nodes = []
        for nd in way.findall('./nd'):
            ref = nd.get('ref')
            if graph.get_node(ref):
                nodes.append(ref)
                new_way.add_node(ref)
        
        # Add way to graph
        graph.add_way(new_way)
        
        # Create edges between consecutive nodes
        for i in range(len(nodes) - 1):
            node1_id = nodes[i]
            node2_id = nodes[i + 1]
            
            node1 = graph.get_node(node1_id)
            node2 = graph.get_node(node2_id)
            
            if node1 and node2:
                # Calculate distance using haversine formula
                dist = haversine(node1.lat, node1.lon, node2.lat, node2.lon)
                # Add both directions (make it an undirected graph)
                graph.add_edge(node1_id, node2_id, dist)
                graph.add_edge(node2_id, node1_id, dist)
    
    print(f"Graph created with {len(graph.nodes)} nodes and {len(graph.ways)} ways")
    return graph

def a_star_algorithm(graph, start_id, goal_id):
    """A* algorithm implementation"""
    if start_id not in graph.nodes or goal_id not in graph.nodes:
        return None
        
    start_node = graph.get_node(start_id)
    goal_node = graph.get_node(goal_id)
    
    # Create heuristic function (straight-line distance)
    def heuristic(node_id):
        node = graph.get_node(node_id)
        return haversine(node.lat, node.lon, goal_node.lat, goal_node.lon)
    
    # Priority queue for A*
    open_set = []
    heapq.heappush(open_set, (0, start_id))
    
    # For path reconstruction
    came_from = {}
    
    # Cost from start to node
    g_score = {node_id: float('inf') for node_id in graph.nodes}
    g_score[start_id] = 0
    
    # Estimated total cost
    f_score = {node_id: float('inf') for node_id in graph.nodes}
    f_score[start_id] = heuristic(start_id)
    
    # Set to keep track of nodes in the open set
    open_set_hash = {start_id}
    
    while open_set:
        _, current_id = heapq.heappop(open_set)
        open_set_hash.remove(current_id)
        
        if current_id == goal_id:
            # Reconstruct path
            path = []
            while current_id in came_from:
                path.append(current_id)
                current_id = came_from[current_id]
            path.append(start_id)
            return path[::-1]  # Reverse the path
        
        current_node = graph.get_node(current_id)
        for neighbor_id in current_node.get_connections():
            # Calculate tentative g_score
            tentative_g_score = g_score[current_id] + current_node.get_weight(neighbor_id)
            
            if tentative_g_score < g_score[neighbor_id]:
                # This path to neighbor is better than any previous one
                came_from[neighbor_id] = current_id
                g_score[neighbor_id] = tentative_g_score
                f_score[neighbor_id] = tentative_g_score + heuristic(neighbor_id)
                
                if neighbor_id not in open_set_hash:
                    heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))
                    open_set_hash.add(neighbor_id)
    
    # If we get here, no path was found
    return None

class OSMPathFinderApp:
    def __init__(self, root, graph):
        self.root = root
        self.graph = graph
        self.canvas_width = 800
        self.canvas_height = 600
        self.selected_points = []
        self.closest_nodes = []
        self.path = None
        
        # Initial zoom and pan settings
        self.zoom_factor = 1.0
        self.center_lat = (self.graph.min_lat + self.graph.max_lat) / 2
        self.center_lon = (self.graph.min_lon + self.graph.max_lon) / 2
        self.view_width = self.graph.max_lon - self.graph.min_lon
        self.view_height = self.graph.max_lat - self.graph.min_lat
        
        # Mouse dragging for panning
        self.drag_start_x = None
        self.drag_start_y = None
        self.is_dragging = False
        
        self.root.title("OSM Path Finder")
        
        self.frame = Frame(root)
        self.frame.pack(fill=tk.BOTH, expand=True)
        
        self.canvas = Canvas(self.frame, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Control frame
        self.control_frame = Frame(self.frame)
        self.control_frame.pack(fill=tk.X)
        
        self.status_label = Label(self.control_frame, text="Click to select starting point")
        self.status_label.pack(side=tk.LEFT, padx=5)
        
        self.reset_button = Button(self.control_frame, text="Reset", command=self.reset)
        self.reset_button.pack(side=tk.RIGHT, padx=5)
        
        # Zoom slider
        self.zoom_frame = Frame(self.frame)
        self.zoom_frame.pack(fill=tk.X)
        
        self.zoom_label = Label(self.zoom_frame, text="Zoom:")
        self.zoom_label.pack(side=tk.LEFT, padx=5)
        
        self.zoom_slider = Scale(self.zoom_frame, from_=1, to=10, orient=tk.HORIZONTAL, 
                                 resolution=0.1, command=self.on_zoom_change)
        self.zoom_slider.set(self.zoom_factor)
        self.zoom_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Event bindings
        self.canvas.bind("<Button-1>", self.on_canvas_click)
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)  # Windows
        self.canvas.bind("<Button-4>", self.on_mouse_wheel)    # Linux scroll up
        self.canvas.bind("<Button-5>", self.on_mouse_wheel)    # Linux scroll down
        self.canvas.bind("<ButtonPress-3>", self.on_right_click_start)  # Right click for drag
        self.canvas.bind("<ButtonRelease-3>", self.on_right_click_end)
        self.canvas.bind("<B3-Motion>", self.on_right_drag)
        
        # Hover information (cursor magnet visual feedback)
        self.hover_marker = None
        self.canvas.bind("<Motion>", self.on_mouse_move)
        
        # Draw the map initially
        self.draw_map()
    
    def draw_map(self):
        """Draw the OSM map on the canvas"""
        self.canvas.delete("all")
        
        # Draw edges (ways)
        for way_id, way in self.graph.ways.items():
            # Draw line segments for each consecutive pair of nodes in the way
            for i in range(len(way.nodes) - 1):
                node1_id = way.nodes[i]
                node2_id = way.nodes[i + 1]
                
                node1 = self.graph.get_node(node1_id)
                node2 = self.graph.get_node(node2_id)
                
                if node1 and node2:
                    x1, y1 = self.geo_to_canvas(node1.lat, node1.lon)
                    x2, y2 = self.geo_to_canvas(node2.lat, node2.lon)
                    
                    # Only draw if at least partially within canvas bounds
                    if (0 <= x1 <= self.canvas_width or 0 <= x2 <= self.canvas_width) and \
                       (0 <= y1 <= self.canvas_height or 0 <= y2 <= self.canvas_height):
                        self.canvas.create_line(x1, y1, x2, y2, fill="gray", width=2, tags="way")
        
        # Draw selected points and path if available
        if self.path:
            # Draw the path
            path_coords = []
            for node_id in self.path:
                node = self.graph.get_node(node_id)
                x, y = self.geo_to_canvas(node.lat, node.lon)
                path_coords.append(x)
                path_coords.append(y)
            
            if len(path_coords) >= 4:  # Need at least 2 points (4 coordinates)
                self.canvas.create_line(path_coords, fill="blue", width=3, tags="path")
        
        # Draw selected points
        for i, point in enumerate(self.selected_points):
            lat, lon = point
            x, y = self.geo_to_canvas(lat, lon)
            color = "green" if i == 0 else "red"
            self.canvas.create_oval(x-6, y-6, x+6, y+6, fill=color, outline="black", width=2, tags="point")
    
    def geo_to_canvas(self, lat, lon):
        """Convert geographic coordinates to canvas coordinates with zoom and pan"""
        # Calculate the bounds of our current view
        half_width = self.view_width / (2 * self.zoom_factor)
        half_height = self.view_height / (2 * self.zoom_factor)
        
        min_lon = self.center_lon - half_width
        max_lon = self.center_lon + half_width
        min_lat = self.center_lat - half_height
        max_lat = self.center_lat + half_height
        
        # Normalize to [0, 1] range within current view
        x_norm = (lon - min_lon) / (max_lon - min_lon) if max_lon > min_lon else 0.5
        # Flip y-axis (latitude) because canvas coordinates increase downward
        y_norm = 1 - (lat - min_lat) / (max_lat - min_lat) if max_lat > min_lat else 0.5
        
        # Scale to canvas size
        x = int(x_norm * self.canvas_width)
        y = int(y_norm * self.canvas_height)
        
        return x, y
    
    def canvas_to_geo(self, x, y):
        """Convert canvas coordinates to geographic coordinates with zoom and pan"""
        # Calculate the bounds of our current view
        half_width = self.view_width / (2 * self.zoom_factor)
        half_height = self.view_height / (2 * self.zoom_factor)
        
        min_lon = self.center_lon - half_width
        max_lon = self.center_lon + half_width
        min_lat = self.center_lat - half_height
        max_lat = self.center_lat + half_height
        
        # Normalize canvas coordinates
        x_norm = x / self.canvas_width
        y_norm = y / self.canvas_height
        
        # Convert to geographic coordinates
        lon = min_lon + x_norm * (max_lon - min_lon)
        lat = max_lat - y_norm * (max_lat - min_lat)
        
        return lat, lon
    
    def find_closest_point_on_road(self, canvas_x, canvas_y):
        """Find the closest point on any road to the given canvas coordinates"""
        lat, lon = self.canvas_to_geo(canvas_x, canvas_y)
        
        closest_dist = float('inf')
        closest_point = None
        closest_node_id = None
        
        # Check all ways (roads)
        for way_id, way in self.graph.ways.items():
            # Check each segment in the way
            for i in range(len(way.nodes) - 1):
                node1_id = way.nodes[i]
                node2_id = way.nodes[i + 1]
                
                node1 = self.graph.get_node(node1_id)
                node2 = self.graph.get_node(node2_id)
                
                if node1 and node2:
                    x1, y1 = self.geo_to_canvas(node1.lat, node1.lon)
                    x2, y2 = self.geo_to_canvas(node2.lat, node2.lon)
                    
                    # Calculate distance from click to this line segment
                    dist, (proj_x, proj_y) = point_to_line_distance(canvas_x, canvas_y, x1, y1, x2, y2)
                    
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_point = self.canvas_to_geo(proj_x, proj_y)
                        
                        # Determine which node is closer to the projection point
                        dist1 = math.sqrt((proj_x - x1)**2 + (proj_y - y1)**2)
                        dist2 = math.sqrt((proj_x - x2)**2 + (proj_y - y2)**2)
                        closest_node_id = node1_id if dist1 < dist2 else node2_id
        
        return closest_point, closest_node_id, closest_dist
    
    def on_mouse_move(self, event):
        """Handle mouse movement for cursor magnet preview"""
        # Remove previous hover marker
        if self.hover_marker:
            self.canvas.delete(self.hover_marker)
            self.hover_marker = None
        
        # Find closest point on any road
        closest_point, _, dist = self.find_closest_point_on_road(event.x, event.y)
        
        # Only show the magnet preview if we're within a reasonable distance of a road
        if closest_point and dist < 50:  # Threshold distance in pixels
            lat, lon = closest_point
            x, y = self.geo_to_canvas(lat, lon)
            self.hover_marker = self.canvas.create_oval(x-4, y-4, x+4, y+4, 
                                                      fill="yellow", outline="black", tags="hover")
    
    def on_canvas_click(self, event):
        """Handle canvas click events with magnetic snapping to roads"""
        if self.is_dragging:
            return
            
        # Find closest point on any road and the associated node
        closest_point, closest_node_id, dist = self.find_closest_point_on_road(event.x, event.y)
        
        if closest_point and closest_node_id:
            lat, lon = closest_point
            
            if len(self.selected_points) < 2:
                self.selected_points.append((lat, lon))
                self.closest_nodes.append(closest_node_id)
                
                if len(self.selected_points) == 1:
                    self.status_label.config(text="Click to select destination point")
                else:
                    self.status_label.config(text="Finding path...")
                    self.find_path()
            
            self.draw_map()
    
    def on_zoom_change(self, value):
        """Handle zoom slider change"""
        new_zoom = float(value)
        self.zoom_factor = new_zoom
        self.draw_map()
    
    def on_mouse_wheel(self, event):
        """Handle mouse wheel for zooming"""
        # Get the direction from the event
        if event.num == 4 or event.delta > 0:  # Scroll up
            self.zoom_in(event.x, event.y)
        elif event.num == 5 or event.delta < 0:  # Scroll down
            self.zoom_out(event.x, event.y)
    
    def zoom_in(self, center_x=None, center_y=None):
        """Zoom in, optionally centered on a specific point"""
        if center_x is None:
            center_x = self.canvas_width / 2
        if center_y is None:
            center_y = self.canvas_height / 2
            
        # Get the geographic coordinates of the center point before zooming
        old_center_lat, old_center_lon = self.canvas_to_geo(center_x, center_y)
        
        # Increase zoom factor
        self.zoom_factor = min(10.0, self.zoom_factor * 1.2)
        self.zoom_slider.set(self.zoom_factor)
        
        # Update the center to keep the point under the cursor at the same position
        self.center_lat = old_center_lat
        self.center_lon = old_center_lon
        
        self.draw_map()
    
    def zoom_out(self, center_x=None, center_y=None):
        """Zoom out, optionally centered on a specific point"""
        if center_x is None:
            center_x = self.canvas_width / 2
        if center_y is None:
            center_y = self.canvas_height / 2
            
        # Get the geographic coordinates of the center point before zooming
        old_center_lat, old_center_lon = self.canvas_to_geo(center_x, center_y)
        
        # Decrease zoom factor
        self.zoom_factor = max(1.0, self.zoom_factor / 1.2)
        self.zoom_slider.set(self.zoom_factor)
        
        # Update the center to keep the point under the cursor at the same position
        self.center_lat = old_center_lat
        self.center_lon = old_center_lon
        
        self.draw_map()
    
    def on_right_click_start(self, event):
        """Start dragging the map"""
        self.is_dragging = True
        self.drag_start_x = event.x
        self.drag_start_y = event.y
    
    def on_right_click_end(self, event):
        """End dragging the map"""
        self.is_dragging = False
    
    def on_right_drag(self, event):
        """Handle map dragging with right mouse button"""
        if not self.is_dragging:
            return
            
        # Calculate the movement in canvas coordinates
        dx = self.drag_start_x - event.x
        dy = self.drag_start_y - event.y
        
        # Convert the movement to geographic coordinates
        half_width = self.view_width / (2 * self.zoom_factor)
        half_height = self.view_height / (2 * self.zoom_factor)
        
        # Calculate the geographic distance per pixel
        lon_per_pixel = (2 * half_width) / self.canvas_width
        lat_per_pixel = (2 * half_height) / self.canvas_height
        
        # Update the center point
        self.center_lon += dx * lon_per_pixel
        self.center_lat -= dy * lat_per_pixel  # Negative because y increases downward in canvas
        
        # Update the drag start point
        self.drag_start_x = event.x
        self.drag_start_y = event.y
        
        self.draw_map()
    
    def find_path(self):
        """Find the shortest path between the two selected points"""
        if len(self.closest_nodes) == 2:
            start_id = self.closest_nodes[0]
            goal_id = self.closest_nodes[1]
            
            self.path = a_star_algorithm(self.graph, start_id, goal_id)
            
            if self.path:
                path_length = 0
                for i in range(len(self.path) - 1):
                    node1 = self.graph.get_node(self.path[i])
                    node2 = self.graph.get_node(self.path[i + 1])
                    path_length += haversine(node1.lat, node1.lon, node2.lat, node2.lon)
                
                self.status_label.config(text=f"Path found! Length: {path_length:.2f} km")
            else:
                self.status_label.config(text="No path found between selected points")
    
    def reset(self):
        """Reset the application state"""
        self.selected_points = []
        self.closest_nodes = []
        self.path = None
        self.status_label.config(text="Click to select starting point")
        self.draw_map()

def main():
    if len(sys.argv) > 1:
        osm_file = sys.argv[1]
    else:
        osm_file = "map.osm"  # Default file name
    
    graph = parse_osm(osm_file)
    
    if not graph:
        print("Failed to create graph from OSM file.")
        return
    
    root = tk.Tk()
    app = OSMPathFinderApp(root, graph)
    root.mainloop()

if __name__ == "__main__":
    main()
