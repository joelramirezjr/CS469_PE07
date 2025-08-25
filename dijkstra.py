from typing import Dict, List, Optional


class DijkstraClass:
    """...write comment about this method..."""
    """This class will compute lowest-cost path from 's' to 'f' using Dijkstra's algorithm.

    The graph is an adjacency map where graph[u][v] is the nonnegative weight
    of the directed edge u -> v, e.g.:
        {
            's': {'a': 6, 'b': 2},
            'a': {'c': 1},
            'b': {'a': 3, 'c': 5},
            'c': {'f': 2},
            'f': {}
        }

    Internal tables:
        costs[u]   -> current best-known total cost to reach u from 's'
        parents[u] -> previous node on the best-known path to u
        processed  -> nodes for which the best cost is finalized
    """

    def __init__(self, graph: Dict[str, Dict[str, float]]):
        """Store the graph and set up empty tables."""

        # Graph to compute shortest path on
        self.graph = graph

        # Define infinity to be used throughout class
        self.infinity = float("inf")

        # Define known/computed costs to node
        self.costs: Dict[str, float] = {}

        # Define parent of each node (used to reconstruct the path)
        self.parents: Dict[str, Optional[str]] = {}

        # Track whether a node has been processed (finalized)
        self.processed: List[str] = []

        # Cached latest reconstructed path for print_path()
        self._last_path: List[str] = []

    def initial_costs_parents(self) -> None:
        """...write comment about this method...
        """
        """Initialize costs and parents according to Dijkstra setup.

        * Every node's cost starts as infinity, except the start node 's' which is 0.
        * Parents are initialized to None.
        * Ensures nodes that appear only as neighbors are also included.
        * Clears the processed list
        """
        # Include all nodes that appear as keys
        nodes = set(self.graph.keys())
        # Include nodes that appear only as neighbors
        for u, nbrs in self.graph.items():
            nodes.update(nbrs.keys())

        self.costs = {node: self.infinity for node in nodes}
        self.parents = {node: None for node in nodes}
        self.processed = []
        self._last_path = []

        # Start node 's' has cost 0 by convention in this assignment
        if 's' in self.costs:
            self.costs['s'] = 0.0

    def find_shorted_path(self) -> List[str]:
        """...write comment about this method...
        """
        """Here we run Dijkstra's algorithm and return the path list from 's' to 'f'.

        The algorithm repeatedly picks the lowest-cost unprocessed node and then updates 
        its neighbors. After processing all nodes, the path
        to 'f' is reconstructed by following parents backwards.
        """
        node = self.find_lowest_cost_node(self.costs)
        while node is not None:
            cost_to_node = self.costs[node]
            # Relax each neighbor of the current node
            for neighbor, weight in self.graph.get(node, {}).items():
                new_cost = cost_to_node + weight
                if new_cost < self.costs.get(neighbor, self.infinity):
                    self.costs[neighbor] = new_cost
                    self.parents[neighbor] = node
            # Mark as processed
            self.processed.append(node)
            # Pick the next unprocessed node with the lowest cost
            node = self.find_lowest_cost_node(self.costs)

        # Reconstruct the path from 's' to 'f'
        path: List[str] = []
        node = 'f'
        while node is not None:
            path.append(node)
            node = self.parents.get(node)
        path.reverse()
        self._last_path = path if path and path[0] == 's' else []
        return self._last_path

    def find_lowest_cost_node(self, costs: Dict[str, float]) -> Optional[str]:
        """...write comment about this method...
        """
        """ Here we find the unprocessed node with the lowest cost so far.
        """
        lowest_cost = self.infinity
        lowest_cost_node: Optional[str] = None
        for node, cost in costs.items():
            if (node not in self.processed) and (cost < lowest_cost):
                lowest_cost = cost
                lowest_cost_node = node
        return lowest_cost_node

    def print_path(self):
        """Print the path from 's' (start) to 'f' (finish).

        If a path exists, it shows the list of nodes in order.
        If no path exists, it prints 'No path to finish'.
        """
        # If no path has been saved yet, try to rebuild one
        if not self._last_path:
            path = []
            # Start at finish and walk backwards through parents
            node = 'f'
            while node is not None:
                path.append(node)
                node = self.parents.get(node)
            path.reverse()

            # Only save the path if it really starts at 's'
            if path and path[0] == 's':
                self._last_path = path

        # Print the result
        if self._last_path:
            print(self._last_path)
        else:
            print("No path to finish")

