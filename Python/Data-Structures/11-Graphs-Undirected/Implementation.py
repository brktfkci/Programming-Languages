class UndirectedGraph:
    def __init__(self):
        self.graph = {}

    def add_vertex(self, vertex):
        if vertex not in self.graph:
            self.graph[vertex] = []

    def add_edge(self, vertex1, vertex2):
        if vertex1 in self.graph and vertex2 in self.graph:
            if vertex2 not in self.graph[vertex1]:
                self.graph[vertex1].append(vertex2)
            if vertex1 not in self.graph[vertex2]:
                self.graph[vertex2].append(vertex1)

    def remove_vertex(self, vertex):
        if vertex in self.graph:
            del self.graph[vertex]
            for v in self.graph:
                if vertex in self.graph[v]:
                    self.graph[v].remove(vertex)

    def remove_edge(self, vertex1, vertex2):
        if vertex1 in self.graph and vertex2 in self.graph:
            if vertex2 in self.graph[vertex1]:
                self.graph[vertex1].remove(vertex2)
            if vertex1 in self.graph[vertex2]:
                self.graph[vertex2].remove(vertex1)

    def search_vertex(self, vertex):
        return vertex in self.graph

    def search_edge(self, vertex1, vertex2):
        if vertex1 in self.graph and vertex2 in self.graph:
            return vertex2 in self.graph[vertex1] and vertex1 in self.graph[vertex2]
        return False

    def get_neighbors(self, vertex):
        if vertex in self.graph:
            return self.graph[vertex]
        return []

# Example usage:
my_graph = UndirectedGraph()

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
print("Search for edge ('A', 'C'):", my_graph.search_edge("A", "C"))

# Removing edges and vertices
my_graph.remove_edge("A", "B")
my_graph.remove_vertex("C")

# Getting neighbors
print("Neighbors of 'A':", my_graph.get_neighbors("A"))
