class TreeNode:
    def __init__(self, data):
        self.data = data
        self.left = None
        self.right = None

class BinarySearchTree:
    def __init__(self):
        self.root = None

    def add(self, data):
        self.root = self._add_recursive(self.root, data)

    def _add_recursive(self, node, data):
        if not node:
            return TreeNode(data)
        if data < node.data:
            node.left = self._add_recursive(node.left, data)
        elif data > node.data:
            node.right = self._add_recursive(node.right, data)
        return node

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

        return node

    def _find_min(self, node):
        while node.left:
            node = node.left
        return node.data

# Example usage:
my_bst = BinarySearchTree()

# Adding items
my_bst.add(10)
my_bst.add(5)
my_bst.add(15)

# Searching items
found_node = my_bst.search(5)
print("Found node with data 5:", found_node.data if found_node else "Not found")

# Removing items
my_bst.remove(10)
found_node = my_bst.search(10)
print("Found node with data 10 after removal:", found_node.data if found_node else "Not found")
