class Solution:
    def group_anagrams(strs):
        result = {}
        for word in strs:
            count = [0] * 26
            for char in word:
                count[ord(char) - ord("a")] = count[ord(char) - ord("a")] + 1        
            if tuple(count) in result.keys():
                result[tuple(count)].append(word)
            else:
                result[tuple(count)] = [word]     
        return result.values()
        
    
strs = ["eat","tea","tan","ate","nat","bat"]
sol = Solution.group_anagrams(strs)
print(sol)






