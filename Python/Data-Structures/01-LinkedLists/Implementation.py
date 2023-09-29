class Node:
    def __init__(self, data):
        self.data = data
        self.next = None

class SinglyLinkedList:
    def __init__(self):
        self.head = None
        
    def add_head(self, data):
        new_node = Node(data)
        new_node.next = self.head
        self.head = new_node

    def remove_head(self):
        if self.head:
            self.head = self.head.next
        else:
            print("List is empty. Cannot remove from an empty list.")
    def append(self, data):
        new_node = Node(data)
        if not self.head:
            self.head = new_node
        else:
            current = self.head
            while current.next:
                current = current.next
            current.next = new_node

    def remove(self, data):
        if not self.head:
            return
        if self.head.data == data:
            self.head = self.head.next
            return
        current = self.head
        while current.next:
            if current.next.data == data:
                current.next = current.next.next
                return
            current = current.next

    def search(self, data):
        current = self.head
        while current:
            if current.data == data:
                return True
            current = current.next
        return False

    def display(self):
        result = []
        current = self.head
        while current:
            result.append(current.data)
            current = current.next
        return result

# Example usage:
my_list = SinglyLinkedList()

# Adding items to the head
my_list.add_head(30)
my_list.add_head(20)
my_list.add_head(10)
print("List after adding items to the head:", my_list.display())

# Removing item from the head
my_list.remove_head()
print("List after removing the head item:", my_list.display())

# Adding items
my_list.append(10)
my_list.append(20)
my_list.append(30)
print("List after adding items:", my_list.display())

# Removing items
my_list.remove(20)
print("List after removing 20:", my_list.display())

# Searching items
print("Search for 30:", my_list.search(30))
print("Search for 40:", my_list.search(40))
