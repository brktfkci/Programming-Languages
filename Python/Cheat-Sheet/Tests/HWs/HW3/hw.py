# -*- coding: utf-8 -*-
"""
Created on Mon Nov  9 23:14:35 2020

@author: Burak Tufekci S017309
"""

import numpy as np

def strListToNpArr (size_arr:int, argvs:list):
    argv_list = np.zeros(shape=(size_arr,size_arr))
    ii = 0
    for line in argvs:
        argv_list[ii] = (np.array(line.split(' '))).astype(np.float)
        ii += 1
    return argv_list

def solutionTest (argvs_np, argvLast_np, result):
    dict_check = {}
    tot_err = 0
    for ii in range(len(argvs_np)):
        check_sum = 0
        for jj in range(len(argvLast_np)):
            check_sum = check_sum + (argvs_np[ii][jj] * result[jj])
        dict_check[ii] = check_sum - argvLast_np[ii]
        tot_err += dict_check[ii]
    return dict_check, tot_err

# read all lines from nov09.txt and add to line_list
f = open('nov09.txt', 'r', encoding='utf-8')
line_list = f.readlines()
f.close()

# define size of array as int
size_arr = int(line_list[1].strip('SIZE: \n'))

# get all line of line_list except two items from the tail as list
argvs = line_list[2:-2]

# convert string list to numpuy array
argvs_np = strListToNpArr(size_arr, argvs)

# get last line of line_list
argvLast_np = (np.array(line_list[-1].split(' '))).astype(np.float)

# solve linear equation 
result = np.linalg.solve(argvs_np, argvLast_np)

# test results 
dict_test, err_test = solutionTest(argvs_np, argvLast_np, result)

# write fancy way to output.txt
f = open('output.txt', 'w', encoding='utf-8')
f.write ('Number of Equations : {}\n'.format(size_arr))
f.write ('EQUATIONS\n')
f.write ('----------\n')

# for dynamic formatting create str_line
str_line = ''
for ii in  range(len(argvs_np)):
    str_line = str_line + '{' + str(ii) + ':+.2f}x' + str(ii + 1) + ' '

# every argvs_np element is formatted
for ii in range(len(argvs_np)):
    args = argvs_np[ii]
    f.write ((str_line*1).format(*args) + '= {:+}'.format(argvLast_np[ii]) + '\n')  
f.write ('SOLUTION:\n')
f.write ('----------\n')
for ii in range(len(result)):
    f.write('x{} = {}\n'.format(ii+1, result[ii]))
f.write('\n')
f.write('Check if the solution satisfies the equations:\n')
for ii,jj in dict_test.items():
    f.write('Equation {0:3d}  : Error is {1}\n'.format(ii+1, jj))
f.write('SOLUTION IS CORRECT WITH A TOTAL ERROR OF {}\n'.format(err_test))
f.close()
