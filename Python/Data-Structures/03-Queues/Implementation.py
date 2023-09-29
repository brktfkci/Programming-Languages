class Queue:
    def __init__(self):
        self.items = []

    def enqueue(self, item):
        self.items.append(item)

    def dequeue(self):
        if not self.is_empty():
            return self.items.pop(0)
        else:
            print("Queue is empty. Cannot dequeue from an empty queue.")
            return None

    def is_empty(self):
        return len(self.items) == 0

    def peek(self):
        if not self.is_empty():
            return self.items[0]
        else:
            print("Queue is empty. Cannot peek at an empty queue.")
            return None

    def search(self, item):
        return item in self.items

    def size(self):
        return len(self.items)

# Example usage:
my_queue = Queue()

# Adding items (enqueue)
my_queue.enqueue(10)
my_queue.enqueue(20)
my_queue.enqueue(30)

# Removing items (dequeue)
dequeued_item = my_queue.dequeue()
print("Dequeued item:", dequeued_item)

# Searching items
print("Search for 20:", my_queue.search(20))
print("Search for 40:", my_queue.search(40))

# Check if the queue is empty
print("Is the queue empty?", my_queue.is_empty())

# Get the size of the queue
print("Size of the queue:", my_queue.size())
