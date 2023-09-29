## Binary Min Heap

A Binary Min-Heap is a binary tree structure that satisfies the heap property, where the parent node's value is smaller than or equal to the values of its children. It's typically used for priority queue operations, where the minimum element can be efficiently extracted.

- Adding Items (insert operation): The insert operation has a worst-case time complexity of `O(log n)`, where `n` is the number of elements in the heap. This is because the insert operation involves swapping elements upwards along the heap's height until the heap property is satisfied, which takes logarithmic time.

- Removing Items (extract_min operation): The extract_min operation, which removes the minimum element from the heap, also has a worst-case time complexity of `O(log n)`. After removing the minimum element, the operation involves swapping elements downwards along the heap's height to restore the heap property.

- Searching for Items (search operation): The search operation, which checks if an element is in the heap, has a time complexity of `O(n)` in the worst case because it may need to check all elements in the heap.

Binary Min-Heaps are efficient for maintaining a minimum element and extracting it quickly, making them suitable for priority queue operations. However, they are not optimized for efficient searching. If searching is a common operation, other data structures like binary search trees or hash tables may be more appropriate.