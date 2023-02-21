# -*- coding: utf-8 -*-
"""
Created on Thu Jun 30 13:12:05 2022

@author: bt0278
"""

nums1 = [1,3]
nums2 = [2,4]

m = len(nums1)
n = len(nums2)

idxLow  = 0
idxHigh = 0
idxMedian = (0,0)

if m % 2 == 0:
    if n % 2 == 0:
        medianOfNums1 = m // 2 
        medianOfNums2 = n // 2 
        idxLow  = medianOfNums1 + medianOfNums2 
        idxHigh = medianOfNums1 + medianOfNums2 + 1    
        idxMedian = (idxLow, idxHigh)
    else:
        medianOfNums1 = m // 2 
        medianOfNums2 = n // 2 + 1
        idxLow  = medianOfNums1 + medianOfNums2 
        idxHigh = medianOfNums1 + medianOfNums2    
        idxMedian = (idxLow, idxHigh)
else:
    if n % 2 == 0:
        medianOfNums1 = m // 2 + 1
        medianOfNums2 = n // 2 
        idxLow  = medianOfNums1 + medianOfNums2 
        idxHigh = medianOfNums1 + medianOfNums2    
        idxMedian = (idxLow, idxHigh)               
    else:
        medianOfNums1 = m // 2 + 1
        medianOfNums2 = n // 2 + 1
        idxLow  = medianOfNums1 + medianOfNums2 - 1
        idxHigh = medianOfNums1 + medianOfNums2
        idxMedian = (idxLow, idxHigh)                
      

ptr1 = 0
ptr2 = 0
for _ in range(idxMedian[1]):
    if nums1[ptr1] > nums2[ptr2]:
        ptr2 += 1
    else:
        ptr1 += 1
            
if idxMedian[0] == idxMedian[1]:   
    if nums1[ptr1] > nums2[ptr2]:
        mean = nums1[ptr1]
    else:
        mean = nums2[ptr2]
else:
    ptr1 = -1
    ptr2 = -1    
    for _ in range(idxMedian[0]):             
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            