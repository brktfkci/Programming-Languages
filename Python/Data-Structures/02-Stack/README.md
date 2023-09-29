## Stack

A stack is a linear data structure that follows the Last-In-First-Out (LIFO) principle. This means that the last item added to the stack is the first one to be removed.

- Adding Items (push operation): The push operation has an average time complexity of `O(1)` because it simply appends an item to the end of the list.

- Removing Items (pop operation): The pop operation has an average time complexity of `O(1)` because it removes the last item from the list, which is a constant-time operation.

- Searching for Items (search operation): The search operation has a time complexity of `O(n)` because it may need to traverse the entire stack to find the item, worst-case scenario.

Stacks are efficient for adding and removing items (push and pop) and checking if they are empty, all of which have constant or near-constant time complexity. Searching for items in a stack is less efficient and has a linear time complexity `O(n)` in the worst case because you may need to traverse the entire stack.