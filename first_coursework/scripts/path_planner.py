#!/usr/bin/env python3

import rospy
from uvcontroller.msg import Sector 

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)

        rospy.Subscriber("/human/sector", Sector, self.human_sector_callback)
        self.human_position = None

        self.graph = {
            'A': ['B', 'F'],
            'B': ['A', 'G', 'C'],
            'C': ['B', 'H', 'D'],
            'D': ['C', 'I', 'E'],
            'E': ['D', 'J'],
            'F': ['A', 'G', 'K'],
            'G': ['F', 'B', 'L', 'H'],
            'H': ['G', 'C', 'M', 'I'],
            'I': ['H', 'N', 'J', 'D'],
            'J': ['I', 'E', 'O'],
            'K': ['F', 'L', 'P'],
            'L': ['K', 'Q', 'M', 'G'],
            'M': ['L', 'R', 'N', 'H'],
            'N': ['M', 'S', 'O', 'I'],
            'O': ['N', 'T', 'J'],
            'P': ['U', 'Q', 'K'],
            'Q': ['P', 'V', 'R', 'L'],
            'R': ['Q', 'W', 'S', 'M'],
            'S': ['R', 'X', 'T', 'N'],
            'T': ['S', 'Y', 'O'],
            'U': ['P', 'V'],
            'V': ['U', 'Q', 'W'],
            'W': ['V', 'R', 'X'],
            'X': ['W', 'S', 'Y'],
            'Y': ['X', 'T']
        }
        
        self.vertex_value = {
            'A': (0, 0),
            'B': (1, 0),
            'C': (2, 0),
            'D': (3, 0),
            'E': (4, 0),
            'F': (0, 1),
            'G': (1, 1),
            'H': (2, 1),
            'I': (3, 1),
            'J': (4, 1),
            'K': (0, 2),
            'L': (1, 2),
            'M': (2, 2),
            'N': (3, 2),
            'O': (4, 2),
            'P': (0, 3),
            'Q': (1, 3),
            'R': (2, 3),
            'S': (3, 3),
            'T': (4, 3),
            'U': (0, 4),
            'V': (1, 4),
            'W': (2, 4),
            'X': (3, 4),
            'Y': (4, 4)
        }

    def remove_node(self, node):
        """
        Removes a node from the graph.
        """
        if node in self.graph:
            del self.graph[node]
        for vertex in self.graph:
            if node in self.graph[vertex]:
                self.graph[vertex].remove(node)

    def depth_first_search(self, start):
        """
        Performs a Depth-First Search (DFS) starting from the given node.
        Returns a list of all nodes visited, including backtracking steps.
        """
        visited_order = [] 

        def dfs_helper(node):
            visited_order.append(node)

            for neighbor in self.graph[node]:
                if neighbor not in visited_order:
                    dfs_helper(neighbor)
                else:
                    visited_order.append(neighbor)

        # Start DFS
        dfs_helper(start)
        return visited_order

    def vertex_to_value(self, vertex_list):
        """
        Converts a list of vertices to their corresponding coordinate values.
        """
        return [self.vertex_value[vertex] for vertex in vertex_list]
    
    def human_sector_callback(self, msg):
        """
        Callback function to update the human's position.
        """
        self.human_position = (msg.x, msg.y)  
        rospy.loginfo(f"Human is in sector: ({msg.x}, {msg.y})")

    def get_human_position(self):
        """
        Returns the current human position as (x, y) coordinates.
        If the position has not been received yet, returns None.
        """
        if self.human_position is None:
            rospy.logwarn("Human position not yet received.")
        return self.human_position
    
    def get_node_from_position(self, position):

        for node, coords in self.vertex_value.items():
            if coords == position:
                return node
        return None  
    
    def run(self):

        rate = rospy.Rate(10)  

        while not rospy.is_shutdown():
            human_pos = self.get_human_position()
            if human_pos:
                print(f"Human is at position: {human_pos}")
                
                human_node = self.get_node_from_position(human_pos)
                if human_node:
                    print(f"Human is at node: {human_node}")
                    
                    self.remove_node(human_node)
                    print(f"Removed node {human_node} from the graph.")
                    
                    start_node = 'A'  
                    dfs_result = self.depth_first_search(start_node)
                    print(f"DFS traversal starting from {start_node}: {dfs_result}")
                    
                    dfs_coords = self.vertex_to_value(dfs_result)
                    print(f"DFS traversal in coordinates: {dfs_coords}")
                    
                    return dfs_coords 
                else:
                    print("No matching node found for the human's position.")
                    return None 
            else:
                print("Waiting for human position...")

            rate.sleep() 

        return None  

if __name__ == "__main__":
    try:
        planner = PathPlanner()
        dfs_coords = planner.run()
        if dfs_coords:
            print("Final DFS coordinates:", dfs_coords)
        else:
            print("No coordinates returned.")
    except rospy.ROSInterruptException:
        pass
