import heapq

def a_star_algorithm(graph, heuristics, start, goal):
    """
    Finds the shortest path between start and goal nodes using the A* algorithm.
    
    :param graph: Dict representing adjacency list {node: {neighbor: cost}}
    :param heuristics: Dict representing straight-line distance estimates to the goal
    :param start: The starting node string/integer
    :param goal: The target destination node string/integer
    :return: Tuple containing the optimal path list and total path cost
    """
    # Priority Queue elements format: (f_score, current_node)
    # The heap always pops the node with the lowest f_score
    open_set = []
    heapq.heappush(open_set, (heuristics[start], start))
    
    # Tracks the absolute shortest distance from start to a given node
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    
    # Tracks the parent map to reconstruct the final shortest route
    came_from = {}
    
    while open_set:
        # Extract node with the lowest estimated total cost (f_score)
        current_f, current_node = heapq.heappop(open_set)
        
        # Stop condition: return path once goal is reached
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            return path[::-1], g_score[goal] # Return reversed path and total cost
            
        # Evaluate neighbors of the current node
        for neighbor, weight in graph[current_node].items():
            tentative_g_score = g_score[current_node] + weight
            
            # If a cheaper path to this neighbor is uncovered
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristics[neighbor]
                
                # Only explore nodes if they show a potential improvement
                if not any(item[1] == neighbor for item in open_set):
                    heapq.heappush(open_set, (f_score, neighbor))
                    
    return None, float('inf') # Return if no path exists

# --- Implementation Example ---
if __name__ == "__main__":
    # Define a weighted graph map
    node_graph = {
        'A': {'B': 1, 'C': 4},
        'B': {'D': 3, 'C': 2},
        'C': {'E': 5},
        'D': {'F': 2, 'G': 4},
        'E': {'G': 3},
        'F': {'G': 1},
        'G': {}
    }

    # Define straight-line estimations to target node 'G'
    heuristic_estimates = {
        'A': 7,
        'B': 6,
        'C': 4,
        'D': 3,
        'E': 3,
        'F': 1,
        'G': 0
    }

    shortest_path, total_cost = a_star_algorithm(node_graph, heuristic_estimates, 'A', 'G')
    
    print(f"Shortest Path: {' -> '.join(shortest_path)}")
    print(f"Total Cost: {total_cost}")
