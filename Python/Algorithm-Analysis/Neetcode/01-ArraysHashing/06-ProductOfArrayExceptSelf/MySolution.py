class Solution:
    def product_of_array_except_self(nums):
        
        list_result = [1] * len(nums)

        prefix = 1
        for ii in range(0,len(nums),1):
            list_result[ii] = prefix
            prefix *= nums[ii]
            
        postfix = 1
        for ii in range(len(nums)-1, -1, -1):
            list_result[ii] *= postfix
            postfix *= nums[ii]
            
        return list_result
            
            
    
        
nums = [1,2,3,4]


sol = Solution.product_of_array_except_self(nums)
print(sol)
