# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 13:03:48 2020

@author: student
"""

#1
# list_q1 = [ii for ii in range(0,20) if ii %2 == 0]
# list_q2 = [sum(list_q1[:ii+1]) for ii in range(0,10)]


#2
# x>5 and x != 6

#3
# a = [5, 4, 3, 2, 1, 0]
# a[0] = 5
# a[-1] = 0
# a[a[-1]] = 5
# a[a[a[a[2]+1]]] = a[a[a[4]]] = a[a[1]] = a[4] = 1
# a[1:3] = [4, 3]
# a[1:3]+a[-1:] = [4, 3, 0]

#4
# a = [1, 2]
# b = []
# b = b + a
# b = b + a
# a.append(3)
# print (b)
# => [1, 2, 1, 2]

#5
# for i in range(-1,7, -2):
#     for j in range (3):
#         print(1, j)
# => No output

#6
# #Get user input
# inpStr = input("Your message:")
# #Your code goes here. Store reversed string in revStr variable
# inpStrSplitted = inpStr.split(" ")
# revStr = ""
# for ii in inpStrSplitted:
#     revStr = revStr + ii[::-1] + " "

#7
# def convertDate(date:int):
#     date_str = str(date)
#     month_int = int(date_str[:2])
#     day_int = int(date_str[2:4])
#     year_int = int(date_str[4:])
#     months = {
#         1: "January",
#         2: "February",
#         3: "March",
#         4: "April",
#         5: "May",
#         6: "June",
#         7: "July",
#         8: "August",
#         9: "September",
#         10: "October",
#         11: "November",
#         12: "December"
#     }
#     str_result = "{} {}, {}".format(months[month_int], day_int, year_int)
#     return str_result

# inpDate = int(input("Enter date:"))
# print(convertDate(inpDate))

#8
# def hwperc(grades:str , points:int):
#     grades_str_list = grades.split(" ")
#     grades_list = [float(ii) for ii in grades_str_list]
#     avg_grades = sum(grades_list) * 100 / points
#     return avg_grades

# avg_result = hwperc("40.5 50 29.5 45", 200)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    