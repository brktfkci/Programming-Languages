## Directed Graph

A directed graph, also known as a digraph, is a type of graph in which edges (connections) between nodes (vertices) have a direction or orientation. Unlike undirected graphs, where edges imply a bidirectional relationship between nodes, directed graphs represent one-way relationships between nodes.

- Adding Vertex (add_vertex operation): Adding a vertex to the graph takes `O(1)` time on average because it involves inserting a key into a dictionary.

- Adding Edge (add_edge operation): Adding an edge from one vertex to another takes `O(1)` time on average because it involves appending to lists in the dictionary.

- Removing Vertex (remove_vertex operation): Removing a vertex from the graph takes `O(V + E) `time, where `V` is the number of vertices, and `E` is the number of edges. In the worst case, you need to remove the vertex and its edges.

- Removing Edge (remove_edge operation): Removing an edge from one vertex to another takes `O(1)` time on average because it involves removing elements from lists in the dictionary.

- Searching Vertex (search_vertex operation): Searching for a vertex takes `O(1)` time on average because it involves checking if a key exists in a dictionary.

- Searching Edge (search_edge operation): Searching for an edge from one vertex to another takes `O(1)` time on average because it involves checking if an element is present in a list in the dictionary.

Applications of directed graphs include modeling various real-world scenarios where relationships have a clear direction, such as representing transportation networks, information flow, social networks with directed friendships (e.g., Twitter follows), dependency graphs in software, and many other systems that involve directed relationships and processes.

Directed graphs are a fundamental concept in graph theory and have numerous applications in computer science, data science, network analysis, and various other fields. They are used to solve problems like finding paths, analyzing flow, and modeling cause-and-effect relationships.