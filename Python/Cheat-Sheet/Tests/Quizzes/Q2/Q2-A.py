# -*- coding: utf-8 -*-
"""
Created on Thu Oct 29 14:41:13 2020

@author: student
"""

list_1 = [1, 2, 3]
list_2 = ['a', 'b', 'c']

list_3 = [str(jj) + ii for jj in list_1 for ii in list_2]
list_4 = [str(ii) + jj for jj in list_2 for ii in list_1]

str_list_3 = "["
for ii in list_3:
    str_list_3 += "{}{}".format(ii, ", " if ii != "3c" else "]")

str_list_4 = "["
for ii in list_4:
    str_list_4 += "{}{}".format(ii, ", " if ii != "3c" else "]")

str_list_5 = "["
for ii in list_1:
    for jj in list_2:
        str_list_5 += "{}{}{}".format(ii, jj, ", ")
        str_list_5 += "{}{}{}".format(jj, ii, "]" if (jj == "c" and ii == 3) else ", ")

