class Stack:
    def __init__(self):
        self.items = []

    def push(self, item):
        self.items.append(item)

    def pop(self):
        if not self.is_empty():
            return self.items.pop()
        else:
            print("Stack is empty. Cannot pop from an empty stack.")
            return None

    def is_empty(self):
        return len(self.items) == 0

    def peek(self):
        if not self.is_empty():
            return self.items[-1]
        else:
            print("Stack is empty. Cannot peek at an empty stack.")
            return None

    def search(self, item):
        return item in self.items

    def size(self):
        return len(self.items)

# Example usage:
my_stack = Stack()

# Adding items (push)
my_stack.push(10)
my_stack.push(20)
my_stack.push(30)

# Removing items (pop)
popped_item = my_stack.pop()
print("Popped item:", popped_item)

# Searching items
print("Search for 20:", my_stack.search(20))
print("Search for 40:", my_stack.search(40))

# Check if the stack is empty
print("Is the stack empty?", my_stack.is_empty())

# Get the size of the stack
print("Size of the stack:", my_stack.size())
