## Queues

A queue is a linear data structure that follows the First-In-First-Out (FIFO) principle. This means that the first item added to the queue is the first one to be removed.

- Adding Items (enqueue operation):
The enqueue operation has an average time complexity of `O(1)` because it simply appends an item to the end of the list.

- Removing Items (dequeue operation):
The dequeue operation has an average time complexity of `O(1)` because it removes the first item from the list, which is a constant-time operation.

- Searching for Items (search operation):
The search operation has a time complexity of `O(n)` because it may need to traverse the entire queue to find the item, worst-case scenario.

Queues are efficient for adding and removing items (enqueue and dequeue) and checking if they are empty, all of which have constant or near-constant time complexity. Searching for items in a queue is less efficient and has a linear time complexity `O(n)` in the worst case because you may need to traverse the entire queue.