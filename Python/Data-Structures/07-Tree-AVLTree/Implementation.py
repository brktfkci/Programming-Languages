class TreeNode:
    def __init__(self, data):
        self.data = data
        self.left = None
        self.right = None
        self.height = 1  # Height of the node

class AVLTree:
    def __init__(self):
        self.root = None

    def add(self, data):
        self.root = self._add_recursive(self.root, data)

    def _add_recursive(self, node, data):
        if not node:
            return TreeNode(data)
        
        if data < node.data:
            node.left = self._add_recursive(node.left, data)
        else:
            node.right = self._add_recursive(node.right, data)
        
        node.height = 1 + max(self._get_height(node.left), self._get_height(node.right))
        
        # Rebalance the tree if necessary
        return self._balance(node)

    def search(self, data):
        return self._search_recursive(self.root, data) if self.root else None

    def _search_recursive(self, node, data):
        if not node:
            return None
        if data == node.data:
            return node
        elif data < node.data:
            return self._search_recursive(node.left, data)
        else:
            return self._search_recursive(node.right, data)

    def remove(self, data):
        self.root = self._remove_recursive(self.root, data)

    def _remove_recursive(self, node, data):
        if not node:
            return node
        
        if data < node.data:
            node.left = self._remove_recursive(node.left, data)
        elif data > node.data:
            node.right = self._remove_recursive(node.right, data)
        else:
            if not node.left:
                return node.right
            elif not node.right:
                return node.left

            min_val = self._find_min(node.right)
            node.data = min_val
            node.right = self._remove_recursive(node.right, min_val)
        
        node.height = 1 + max(self._get_height(node.left), self._get_height(node.right))
        
        # Rebalance the tree if necessary
        return self._balance(node)

    def _get_height(self, node):
        return node.height if node else 0

    def _balance_factor(self, node):
        return self._get_height(node.left) - self._get_height(node.right)

    def _balance(self, node):
        if self._balance_factor(node) > 1:
            if self._balance_factor(node.left) < 0:
                node.left = self._rotate_left(node.left)
            return self._rotate_right(node)
        elif self._balance_factor(node) < -1:
            if self._balance_factor(node.right) > 0:
                node.right = self._rotate_right(node.right)
            return self._rotate_left(node)
        return node

    def _rotate_left(self, node):
        new_root = node.right
        node.right = new_root.left
        new_root.left = node

        node.height = 1 + max(self._get_height(node.left), self._get_height(node.right))
        new_root.height = 1 + max(self._get_height(new_root.left), self._get_height(new_root.right))

        return new_root

    def _rotate_right(self, node):
        new_root = node.left
        node.left = new_root.right
        new_root.right = node

        node.height = 1 + max(self._get_height(node.left), self._get_height(node.right))
        new_root.height = 1 + max(self._get_height(new_root.left), self._get_height(new_root.right))

        return new_root

    def _find_min(self, node):
        while node.left:
            node = node.left
        return node.data

# Example usage:
my_avl_tree = AVLTree()

# Adding items
my_avl_tree.add(10)
my_avl_tree.add(5)
my_avl_tree.add(15)

# Searching items
found_node = my_avl_tree.search(5)
print("Found node with data 5:", found_node.data if found_node else "Not found")

# Removing items
my_avl_tree.remove(10)
found_node = my_avl_tree.search(10)
print("Found node with data 10 after removal:", found_node.data if found_node else "Not found")
