## Binary Search Tree

A Binary Search Tree (BST) is a binary tree data structure where each node has at most two children, and the values in the left subtree are less than or equal to the value of the node, and the values in the right subtree are greater than the value of the node.

- Adding Items (add operation): Adding an item to a BST takes `O(h)` time, where h is the height of the tree. In the worst case, if the tree is skewed (all nodes are in one subtree), the height can be equal to the number of nodes, resulting in a worst-case time complexity of `O(n)`.

- Removing Items (remove operation): The removal operation also takes `O(h)` time. It can be `O(n)` in the worst case if the tree is skewed.

- Searching for Items (search operation): Searching for an item in a BST also takes `O(h)` time, which can be `O(n)` in the worst case if the tree is skewed.

To improve the worst-case time complexity for adding, removing, and searching, we can consider using a self-balancing binary search tree (e.g., AVL tree or Red-Black tree). These trees ensure that the height remains balanced, resulting in `O(logn)` worst-case time complexity for these operations.
