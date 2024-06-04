class Solution:

    def is_palindrome(s: str) -> bool:
    s = ''.join(c.lower() for c in s if c.isalnum())  
    return s == s[::-1]
           
s = "A man, a plan, a canal: Panama"
sol = Solution.is_palindrome(s)

print(sol)
