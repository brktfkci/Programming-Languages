class MaxHeap:
    def __init__(self):
        self.heap = []

    def insert(self, value):
        self.heap.append(value)
        self._heapify_up(len(self.heap) - 1)

    def extract_max(self):
        if len(self.heap) == 0:
            return None

        if len(self.heap) == 1:
            return self.heap.pop()

        max_value = self.heap[0]
        self.heap[0] = self.heap.pop()
        self._heapify_down(0)

        return max_value

    def search(self, value):
        return value in self.heap

    def _heapify_up(self, index):
        parent_index = (index - 1) // 2
        while index > 0 and self.heap[index] > self.heap[parent_index]:
            self.heap[index], self.heap[parent_index] = self.heap[parent_index], self.heap[index]
            index = parent_index
            parent_index = (index - 1) // 2

    def _heapify_down(self, index):
        while True:
            left_child_index = 2 * index + 1
            right_child_index = 2 * index + 2
            largest = index

            if (left_child_index < len(self.heap) and
                    self.heap[left_child_index] > self.heap[largest]):
                largest = left_child_index

            if (right_child_index < len(self.heap) and
                    self.heap[right_child_index] > self.heap[largest]):
                largest = right_child_index

            if largest == index:
                break

            self.heap[index], self.heap[largest] = self.heap[largest], self.heap[index]
            index = largest

# Example usage:
my_heap = MaxHeap()

# Inserting items
my_heap.insert(5)
my_heap.insert(10)
my_heap.insert(3)
my_heap.insert(8)

# Searching items
print("Search for 10:", my_heap.search(10))
print("Search for 7:", my_heap.search(7))

# Extracting the maximum item
max_item = my_heap.extract_max()
print("Maximum item extracted:", max_item)
