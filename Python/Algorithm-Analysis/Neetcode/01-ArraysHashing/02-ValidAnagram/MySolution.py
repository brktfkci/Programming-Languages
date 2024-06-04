class Solution:
    def is_anagram(s: str, t: str) -> bool:
        dict_s = {}
        dict_t = {}
        for char in s:
            dict_s[char] = 1 + dict_s.get(char, 0)
        for char in t:
            dict_t[char] = 1 + dict_t.get(char, 0)
        return dict_s == dict_t


str_s = "ab"
str_t = "a"

sol = Solution.is_anagram(str_s, str_t)