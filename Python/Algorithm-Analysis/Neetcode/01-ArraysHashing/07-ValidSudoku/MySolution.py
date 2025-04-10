import collections

class Solution:
    def is_valid_sudoku(board: list[list[str]]) -> bool:
        
        cols = collections.defaultdict(set)
        rows = collections.defaultdict(set)
        squares = collections.defaultdict(set)           
        
        for row in range(9):
            for col in range(9):
                if board[row][col] == ".":
                    continue
                if board[row][col] in rows[row] or board[row][col] in cols[col] or board[row][col] in squares[(row//3, col//3)]:
                    return False
                cols[col].add(board[row][col])
                rows[row].add(board[row][col])
                squares[(row//3, col//3)].add(board[row][col])
        return True
            
        
        


        
board = [["1","2",".",".","3",".",".",".","."],
["4",".",".","5",".",".",".",".","."],
[".","9","8",".",".",".",".",".","3"],
["5",".",".",".","6",".",".",".","4"],
[".",".",".","8",".","3",".",".","5"],
["7",".",".",".","2",".",".",".","6"],
[".",".",".",".",".",".","2",".","."],
[".",".",".","4","1","9",".",".","8"],
[".",".",".",".","8",".",".","7","9"]]
 
sol = Solution.is_valid_sudoku(board)
print(sol)
