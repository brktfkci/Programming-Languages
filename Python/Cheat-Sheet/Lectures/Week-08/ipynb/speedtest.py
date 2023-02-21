#SPEED comparison
# importing required packages
import numpy as np
import time
 
# size of arrays and lists
size = 100000000  
 
# declaring lists
list1 = range(size)
list2 = range(size)
# declaring arrays
array1 = np.arange(size)  
array2 = np.arange(size)
 
# list
initialTime = time.time()
resultantList = [(a * b) for a, b in zip(list1, list2)]
 
# calculating execution time
print("Time taken by Lists :", 
      (time.time() - initialTime),
      "seconds")
 
# NumPy array
initialTime = time.time()
resultantArray = array1 * array2
# calculating execution time 
print("Time taken by NumPy Arrays :",
      (time.time() - initialTime),
      "seconds")