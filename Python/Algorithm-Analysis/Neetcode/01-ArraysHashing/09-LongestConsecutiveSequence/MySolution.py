class Solution:

    def longest_consecutive(nums: list[int]) -> int:
        set_nums = set(nums)
        len_most = 0
        
        for num in nums:
            if num-1 not in set_nums:
                len_start = 0
                while num + len_start in set_nums:
                    len_start += 1
                
                len_most = max (len_start, len_most)
                
        return len_most

        

                    

nums = [2,20,4,10,3,4,5] 

sol = Solution.longest_consecutive(nums)

print(sol)
