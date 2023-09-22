class Solution(object):
    def lengthOfLongestSubstring(self, s):
        """
        :type s: str
        :rtype: int
        """
        
        maxLen = 0
        setElements = set()
        ptr1 = 0
        ptr2 = 0
        
        while ptr1 < len(s):
            if s[ptr1] not in setElements:
                setElements.add(s[ptr1])
            else:
                while s[ptr1] in setElements:
                    setElements.remove(s[ptr2])
                    ptr2 += 1
                ptr1 -= 1
        
            if (ptr1 - ptr2 + 1) > maxLen:
                maxLen = ptr1 - ptr2 + 1
            
            ptr1 += 1
                    
        return maxLen