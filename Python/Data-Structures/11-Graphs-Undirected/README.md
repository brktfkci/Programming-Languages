## Undirected Graph

An undirected graph is a type of graph in which edges (connections) between nodes (vertices) have no direction. In other words, if there is an edge connecting vertex A to vertex B in an undirected graph, it also implies that there is an edge connecting vertex B to vertex A.

- Adding Vertex (add_vertex operation): Adding a vertex to the graph takes `O(1)` time because it involves inserting a key into a dictionary.

- Adding Edge (add_edge operation): Adding an edge between two vertices takes `O(1)` time because it involves appending to lists in the dictionary.

- Removing Vertex (remove_vertex operation): Removing a vertex from the graph takes `O(V + E)` time, where `V` is the number of vertices, and `E` is the number of edges. In the worst case, you need to remove the vertex and its edges.

- Removing Edge (remove_edge operation): Removing an edge between two vertices takes `O(1)` time on average because it involves removing elements from lists in the dictionary.

- Searching Vertex (search_vertex operation): Searching for a vertex takes `O(1)` time on average because it involves checking if a key exists in a dictionary.

- Searching Edge (search_edge operation): Searching for an edge between two vertices takes `O(1)` time on average because it involves checking if elements are present in lists in the dictionary.

Applications of undirected graphs include modeling social networks, representing relationships between objects in various domains, solving problems in network design, and graph theory algorithms like finding connected components, calculating shortest paths, and more.