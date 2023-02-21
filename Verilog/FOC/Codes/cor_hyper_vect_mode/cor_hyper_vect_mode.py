###################################################################
######## Cordic HyperBolic Vectoring Mode Implementation ##########
###################################################################
##### xx initial = xx     >>  xn = sqrt(x^2 - y^2)         ########
##### yy initial = yy     >>  yn = 0                       ########
##### zz initial = 0      >>  zn = zz + arctanh(yy/xx)     ########
###################################################################
import math

it_num = 15    ## iteration number
xx = 1700     ## xx must be greater than
yy = 800      ## 
theta = 0      ## give radian value
	
def function_cordic (xx_cor, yy_cor, zz_cor, it_num_cor):
    Ki = 0.8281593609602   ## iteration constant
    # Ki = Km_calc(it_num_cor)   ## iteration constant
    list_degree_circ = [32768, 19344, 10221, 5188, 2604, 1303, 652, 326, 163, 81, 41, 20, 10, 5, 3, 1]
    zz_cor = int(32768 * 4 * (zz_cor / math.pi))
    xx_cor_list = []
    yy_cor_list = []
    zz_cor_list = []
    if ((yy_cor) < 0):
        di = 1
    else:
        di = -1
    print(xx_cor)
    print(yy_cor)
    print(di)
    ii = 1
    while (ii < it_num_cor):
        if(ii == 4 or ii == 13):
            xx_cor_next = xx_cor + (di * (yy_cor >> ii))
            yy_cor_next = yy_cor + (di * (xx_cor >> ii))
            zz_cor_next = di
            xx_cor = xx_cor_next
            yy_cor = yy_cor_next
            zz_cor = zz_cor_next       
            xx_cor_list.append(xx_cor)
            yy_cor_list.append(yy_cor)
            zz_cor_list.append(zz_cor)                   
            xx_cor_next = xx_cor + (di * (yy_cor >> ii))
            yy_cor_next = yy_cor + (di * (xx_cor >> ii))
            zz_cor_next = di
        else:
            xx_cor_next = xx_cor + (di * (yy_cor >> ii))
            yy_cor_next = yy_cor + (di * (xx_cor >> ii))     
            zz_cor_next = di    
        if ((yy_cor_next) < 0):
            di = 1
        else:
            di = -1
        xx_cor = xx_cor_next
        yy_cor = yy_cor_next
        zz_cor = zz_cor_next

        xx_cor_list.append(xx_cor)
        yy_cor_list.append(yy_cor)
        zz_cor_list.append(zz_cor)

        ii = ii + 1

    # x_val_list = [int(element / (Ki)) for element in xx_cor_list]
    # y_val_list = [int(element / Ki / 2) for element in yy_cor_list]
    # z_val_list = [int(element / Ki / 2) for element in zz_cor_list]


    x_val_list = xx_cor_list
    y_val_list = yy_cor_list
    z_val_list = zz_cor_list

    return x_val_list, y_val_list, z_val_list
	
def Ki_calc (num_iter):
	result_calc = 1
	for ii in range (num_iter):
		result_calc = result_calc * (math.sqrt(1+(2**(-2*ii))))
	return result_calc
    
def Km_calc (num_iter):
    result_calc = 1
    for ii in range (num_iter):
        result_calc = result_calc * (math.sqrt(1+(-2**(-2*ii))))
        print(result_calc)
    return result_calc

def rad2deg(rad):
    pi = math.pi
    degree = (rad * 180) / pi
    return degree

def deg2rad(deg):
    pi = math.pi
    radian = deg * (pi / 180)
    return radian

xx_list, yy_list, theta_list = function_cordic(xx, yy, theta, it_num)	

print(xx_list)
print(yy_list)
print(theta_list)

# ii = 1
# while (ii < 15):
#     print(rad2deg(math.atanh(2 ** (-ii))))
#     ii = ii + 1

# print("END !")