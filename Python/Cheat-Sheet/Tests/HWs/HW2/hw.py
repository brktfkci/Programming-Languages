# -*- coding: utf-8 -*-
"""
Created on Mon Nov  2 18:51:48 2020

@author: Burak Tufekci S017309
"""

def checkBarcode (var:str):
    list_num_str = [ii for ii in var]
    try:
        list_num = [int(ii) for ii in list_num_str]
        list_odd = [list_num[ii] for ii in range(0,len(list_num),2)]
        list_even = [list_num[ii] for ii in range(1,len(list_num)-1,2)]
        mod_sum_even = sum(list_even) % 10
        mod_sum_odd = sum(list_odd) % 10
        check1 = 10 - (((3 * mod_sum_odd) + mod_sum_even) % 10)
        if check1 == list_num[-1] and len(list_num) == 12:
            res_bool = True
        else:
            res_bool = False
    except ValueError:
        res_bool = False
    return res_bool


f_read = open("barcodes.txt", mode = 'r', encoding = 'utf-8')
list_str = f_read.read().splitlines()
f_read.close()

f_write = open("output.txt", mode = 'w', encoding = 'utf-8')
for str_val in list_str:
    bool_val = checkBarcode(str_val)
    f_write.write("{} is {} 10-digits barcode\n".format(str_val, "a valid" if bool_val else "an invalid"))
f_write.close()
