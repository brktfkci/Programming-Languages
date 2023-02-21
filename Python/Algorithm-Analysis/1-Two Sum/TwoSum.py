class Solution(object):
    def twoSum(self, nums, target):
        """
        :type nums: List[int]
        :type target: int
        :rtype: List[int]
        """
        setNums = {}
        setNums[target-nums[0]] = 0
        
        for idx, item in enumerate(nums[1:], start=1):
            if item in setNums:
                listIdx = [idx, setNums[item]]
                return listIdx 
            setNums[target-item] = idx
        return None


def testSolution():
    sol = Solution()
    listNums = [[2,7,11,15], 
                [3,2,4],
                [3,3]]
    listTargets = [9,6,6]
    listResults = [[0,1], [1,0], [1,2], [2,1], [0,1], [1,0]]
    resPtr = 0
    for idx, nums in enumerate(listNums):
        target = listTargets[idx]
        result = sol.twoSum(nums, target)
        if result == listResults[resPtr] or result == listResults[resPtr+1]:
            print("Test {} passed".format(idx))
        else:
            print("Test {} not passed!".format(idx))
        resPtr += 2
        
testSolution()        