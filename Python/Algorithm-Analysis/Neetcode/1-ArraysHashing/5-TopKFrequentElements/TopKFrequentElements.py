class Solution:
    def topKFrequent(nums, k):
        
        dict_num = {}
        
        for num in nums:
            if num not in dict_num.keys():
                dict_num[num] = 1
            else:
                dict_num[num] += 1
        
        dict_num_swapped = {value: key for key, value in dict_num.items()}
        
        list_num = []
        for key in sorted(dict_num_swapped.keys(), reverse=True):
            list_num.append(dict_num_swapped[key])
            k -= 1
            if k == 0:
                break
                

        return list_num
        
nums = [1,2]
k = 2

sol = Solution.topKFrequent(nums, k)
