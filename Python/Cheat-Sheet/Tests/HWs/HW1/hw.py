# -*- coding: utf-8 -*-
"""
Created on Thu Oct 29 15:40:20 2020

@author: Burak Tufekci S017309
"""
import math

def findRoots (a, b, c):
    delta = b**2 - 4*a*c
    if delta < 0:
        print("No real root")
    else:
        if a != 0:
            x1 = (-b + math.sqrt(delta)) / 2*a
            x2 = (-b - math.sqrt(delta)) / 2*a
        else:
            x1 = -c / b
            x2 = None
    return x1, x2

def findWriteListOfRoots(lst_coef):
    with open("roots.txt", mode = "w", encoding = "utf-8") as file:
        for coeff in lst_coef:
            if len(coeff) != 3:
                file.write("Equation is not valid !\n")
            else:
                x_1, x_2 = findRoots(int(coeff[0]), int(coeff[1]), int(coeff[2]))
                file.write("The roots of the equation {}x^2 + {}x + {} are {} {} \n".format(coeff[0], coeff[1], coeff[2], x_1, x_2))
    file.close()
    return 

def readFile(fileName):
    list_coeff_str = []
    with open(fileName, mode = "r", encoding = "utf-8" ) as f:
        list_coeff_str = f.read().splitlines() 
    f.close()
    
    list_coeff = []
    for coeff in list_coeff_str:
        list_coeff.append(coeff.split(","))
    
    return list_coeff

findWriteListOfRoots(readFile("coeff.txt"))
