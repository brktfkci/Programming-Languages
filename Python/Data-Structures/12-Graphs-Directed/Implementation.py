class DirectedGraph:
    def __init__(self):
        self.graph = {}

    def add_vertex(self, vertex):
        if vertex not in self.graph:
            self.graph[vertex] = []

    def add_edge(self, from_vertex, to_vertex):
        if from_vertex in self.graph and to_vertex in self.graph:
            if to_vertex not in self.graph[from_vertex]:
                self.graph[from_vertex].append(to_vertex)

    def remove_vertex(self, vertex):
        if vertex in self.graph:
            del self.graph[vertex]
            for v in self.graph:
                if vertex in self.graph[v]:
                    self.graph[v].remove(vertex)

    def remove_edge(self, from_vertex, to_vertex):
        if from_vertex in self.graph and to_vertex in self.graph:
            if to_vertex in self.graph[from_vertex]:
                self.graph[from_vertex].remove(to_vertex)

    def search_vertex(self, vertex):
        return vertex in self.graph

    def search_edge(self, from_vertex, to_vertex):
        if from_vertex in self.graph and to_vertex in self.graph:
            return to_vertex in self.graph[from_vertex]
        return False

    def get_neighbors(self, vertex):
        if vertex in self.graph:
            return self.graph[vertex]
        return []

# Example usage:
my_graph = DirectedGraph()

# Adding vertices
my_graph.add_vertex("A")
my_graph.add_vertex("B")
my_graph.add_vertex("C")

# Adding edges
my_graph.add_edge("A", "B")
my_graph.add_edge("B", "C")
my_graph.add_edge("C", "A")

# Searching vertices and edges
print("Search for vertex 'A':", my_graph.search_vertex("A"))
print("Search for edge ('A', 'B'):", my_graph.search_edge("A", "B"))
print("Search for edge ('B', 'A'):", my_graph.search_edge("B", "A"))

# Removing edges and vertices
my_graph.remove_edge("A", "B")
my_graph.remove_vertex("C")

# Getting neighbors
print("Neighbors of 'A':", my_graph.get_neighbors("A"))
