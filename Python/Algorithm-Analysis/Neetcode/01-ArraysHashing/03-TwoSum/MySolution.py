class Solution:
    def two_sum(nums, target): 
        dict_num = {}
        for ii in range(len(nums)):
            if target - nums[ii] not in dict_num.keys():
                dict_num[nums[ii]] = ii
            else:
                return [dict_num[target - nums[ii]], ii]
            
        
nums = [3,2,4]
target = 6

sol = Solution.two_sum(nums, target)