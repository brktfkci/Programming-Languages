## Valid Sudoku (Leetcode Question 36)

### Question
Determine if a `9 x 9` Sudoku board is valid. Only the filled cells need to be validated **according to the following rules**:

1. Each row must contain the digits `1-9` without repetition.

2. Each column must contain the digits `1-9` without repetition.

3. Each of the nine `3 x 3` sub-boxes of the grid must contain the digits `1-9` without repetition.


**Note**:

- A Sudoku board (partially filled) could be valid but is not necessarily solvable.

- Only the filled cells need to be validated according to the mentioned rules.

![alt text](https://github.com/brktfkci/Programming-Languages/tree/main/Python/Algorithm-Analysis/Neetcode/01-ArraysHashing/07-ValidSudoku/img/img.png)


**Constraints:**

- `board.length == 9`

- `board[i].length == 9`

- `board[i][j]` is a digit `1-9` or `-`.