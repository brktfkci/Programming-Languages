class Solution:
    def isAnagram(s: str, t: str) -> bool:
        dict_s = {}
        dict_t = {}
        for char in s:
            if char in dict_s.keys():
                dict_s[char] += 1 
            else:
                dict_s[char] = 1
        
        for char in t:
            if char in dict_t.keys():
                dict_t[char] += 1 
            else:
                dict_t[char] = 1
        
        return dict_s == dict_t


str_s = "ab"
str_t = "a"

sol = Solution.isAnagram(str_s, str_t)