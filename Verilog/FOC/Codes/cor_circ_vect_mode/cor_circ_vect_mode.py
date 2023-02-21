########################################################
##### Rectangular Coordinate to Polar Coordinate   #####
########################################################
##### xx initial = x     >>>  xn = sqrt(x^2 + y^2) #####
##### yy initial = y     >>>  yn = 0               #####
##### zz initial = 0     >>>  zn = arctan(y/x)     #####
########################################################
import math

it_num = 15             ## iteration number
xx = 600
yy = 800
theta = 0    ## give radian value
	
def function_cordic (xx_cor, yy_cor, zz_cor, it_num_cor):
    list_degree_circ = [32768, 19344, 10221, 5188, 2604, 1303, 652, 326, 163, 81, 41, 20, 10, 5, 3, 1]
    Ki = Ki_calc(it_num_cor)
    zz_cor = int(32768 * 4 * (zz_cor / math.pi))
    print(Ki)

    xx_cor_list = []
    yy_cor_list = []
    zz_cor_list = []
    di = -1

    ii = 0
    while (ii < it_num_cor):
        xx_cor_next = xx_cor - (di * (yy_cor >> ii))
        yy_cor_next = yy_cor + (di * (xx_cor >> ii))
        zz_cor_next = zz_cor - (di * list_degree_circ[ii])
        if (yy_cor_next > 0):
            di = -1
        else:
            di = 1
        xx_cor = xx_cor_next
        yy_cor = yy_cor_next
        zz_cor = zz_cor_next

        xx_cor_list.append(xx_cor)
        yy_cor_list.append(yy_cor)
        zz_cor_list.append(zz_cor)

        ii = ii + 1

    x_val_list = [int(element / Ki) for element in xx_cor_list]
    # x_val_list = xx_cor_list
    y_val_list = yy_cor_list
    z_val_list = zz_cor_list


    return x_val_list, y_val_list, z_val_list
	
def Ki_calc (num_iter):
	result_calc = 1
	for ii in range (num_iter):
		result_calc = result_calc * (math.sqrt(1+(2**(-2*ii))))
	return result_calc


xx_list, yy_list, theta_list = function_cordic(xx, yy, theta, it_num)	

print(xx_list)
print(yy_list)
print(theta_list)