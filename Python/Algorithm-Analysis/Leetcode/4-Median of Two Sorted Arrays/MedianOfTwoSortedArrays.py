class Solution(object):
    def findMedianSortedArrays(self, nums1, nums2):
        """
        :type nums1: List[int]
        :type nums2: List[int]
        :rtype: float
        """
        
        m = len(nums1)
        n = len(nums2)
        
        idxLow  = 0
        idxHigh = 0
        idxMean = (0,0)
        
        if m % 2 == 0:
            if n % 2 == 0:
                medianOfNums1 = m // 2 
                medianOfNums2 = n // 2 
                idxLow  = medianOfNums1 + medianOfNums2 
                idxHigh = medianOfNums1 + medianOfNums2 + 1    
                idxMean = (idxLow, idxHigh)
            else:
                medianOfNums1 = m // 2 
                medianOfNums2 = n // 2 + 1
                idxLow  = medianOfNums1 + medianOfNums2 
                idxHigh = medianOfNums1 + medianOfNums2    
                idxMean = (idxLow, idxHigh)
        else:
            if n % 2 == 0:
                medianOfNums1 = m // 2 + 1
                medianOfNums2 = n // 2 
                idxLow  = medianOfNums1 + medianOfNums2 
                idxHigh = medianOfNums1 + medianOfNums2    
                idxMean = (idxLow, idxHigh)               
            else:
                medianOfNums1 = m // 2 + 1
                medianOfNums2 = n // 2 + 1
                idxLow  = medianOfNums1 + medianOfNums2 - 1
                idxHigh = medianOfNums1 + medianOfNums2
                idxMean = (idxLow, idxHigh)                
                
        if idxMean[0] == idxMean[1]:
            ptr1 = 0
            ptr2 = 0
            for _ in range(idxMean[0]):
                if nums1[ptr1] > nums2[ptr2]:
                    ptr2 += 1
                else:
                    ptr1 += 1
        