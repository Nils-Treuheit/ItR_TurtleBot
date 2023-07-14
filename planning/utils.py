from polygonal_roadmaps import geometry
from shapely.geometry import Point, LineString
import numpy as np

def rrt_connect(start, goal, samples, occupied_space, vm, max_iterations=10000, step_size=0.1):
    # Initialize the trees
    tree_start = Tree(start, occupied_space, vm)
    tree_goal = Tree(goal, occupied_space, vm)

    for _ in range(max_iterations):
        # Randomly sample a point
        sample = samples[np.random.choice(len(samples))]

        # Extend the tree towards the sample
        new_node_start = tree_start.extend(sample, step_size)
        new_node_goal = tree_goal.extend(sample, step_size)

        # Check if the trees have connected
        if new_node_start and tree_goal.is_reachable(new_node_start):
            path = tree_start.get_path(new_node_start) + tree_goal.get_path(new_node_start)
            new_path = list()
            for it in range(len(path)-1):
                part = vm.tuples_to_path([(*path[it],0.0), (*path[it+1],0.0)])
                new_path.extend(part)
            return new_path

        if new_node_goal and tree_start.is_reachable(new_node_goal):
            path = tree_goal.get_path(new_node_goal) + tree_start.get_path(new_node_goal)
            new_path = list()
            for it in range(len(path)-1):
                part = vm.tuples_to_path([(*path[it],0.0), (*path[it+1],0.0)])
                new_path.extend(part)
            return new_path

    # No path found within the maximum number of iterations
    return None


class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent


class Tree:
    def __init__(self, root, occupied_space, vm):
        self.root = Node(root)
        self.nodes = [self.root]
        self.occupied_space = occupied_space
        self.vm = vm

    def extend(self, sample, step_size):
        nearest_node = self.get_nearest_node(sample)
        new_node = self.steer(nearest_node, sample, step_size)

        if new_node and not self.is_colliding(new_node, nearest_node, step_size):
            self.nodes.append(new_node)
            new_node.parent = nearest_node
            return new_node

        return None

    def get_nearest_node(self, sample):
        nearest_node = self.root
        nearest_distance = self.get_distance(nearest_node.position, sample)

        for node in self.nodes:
            distance = self.get_distance(node.position, sample)
            if distance < nearest_distance:
                nearest_node = node
                nearest_distance = distance

        return nearest_node

    def steer(self, from_node, to_position, step_size):
        direction = self.get_direction(from_node.position, to_position)
        new_position = self.get_new_position(from_node.position, direction, step_size)
        return Node(new_position)

    def is_reachable(self, node):
        current_node = node
        while current_node.parent:
            if self.is_colliding(current_node, current_node.parent):
                return False
            current_node = current_node.parent
        return True

    def get_path(self, node):
        path = []
        current_node = node
        while current_node:
            path.append(current_node.position)
            current_node = current_node.parent
        path.reverse()
        return path

    def is_colliding(self, node1, node2):
        line = path_2d(self.vm.tuples_to_path([(*node1.position,0.0), (*node2.position,0.0)]))
        return self.occupied_space.intersects(line)

    @staticmethod
    def get_distance(position1, position2):
        return np.linalg.norm(np.array(position1) - np.array(position2))

    @staticmethod
    def get_direction(from_position, to_position):
        vector = np.array(to_position) - np.array(from_position)
        normalized_vector = vector / np.linalg.norm(vector)
        return normalized_vector.tolist()

    @staticmethod
    def get_new_position(position, direction, step_size):
        new_position = np.array(position) + np.array(direction) * step_size
        return new_position.tolist()

def calculate_occupied_space(map_file, inflation=0.15, simplification=0.05):
    _, o = geometry.read_obstacles(map_file)
    o = o.buffer(inflation)
    o = o.simplify(simplification)
    return o
    
def path_2d(path):
    return LineString(path)


if __name__ == "__main__":
    pass
