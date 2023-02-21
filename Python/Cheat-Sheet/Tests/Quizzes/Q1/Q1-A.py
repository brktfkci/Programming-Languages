# -*- coding: utf-8 -*-
"""
Created on Mon Oct 12 10:21:39 2020

@author: Burak Tufekci

The program determines that the given number is Fibonacci number or not between 0 and 10000
"""
# Take input as integer
num = int(input("Give me a number: "))
# The first two fibonacci number as list
list_fib = [0,1]
# If the added number on list does not match, in order to give information to user I used flag 
flag = False
ii = 2
while list_fib[ii-1] < 10000:
    if(num in list_fib):
        print("The number ", num, " is a Fibonacci number between 0 and 10000", sep='')
        flag = True
        break
    list_fib.append(list_fib[ii-1] + list_fib[ii-2])
    ii += 1
if (not flag):
    print("The number ", num, " is not a Fibonacci number between 0 and 10000", sep='')

