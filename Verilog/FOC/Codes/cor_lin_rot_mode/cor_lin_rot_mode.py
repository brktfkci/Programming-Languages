########################################################
## CORDIC Linear Rotational Mode (small val == y )!!! ##
########################################################
##### xx initial = x     >>>  xn = xx              #####
##### yy initial = y     >>>  yn = yy + xx * zz    #####
##### zz initial = 0     >>>  zn = zz              #####
########################################################
import math

it_num = 15   ## iteration number
xx = 2000
yy = 30
theta = 10     ## give radian value
	
def function_cordic (xx_cor, yy_cor, zz_cor, it_num_cor):
    Ki = Ki_calc(it_num_cor)

    xx_cor_list = []
    yy_cor_list = []
    zz_cor_list = []
    di = -1

    ii = 0
    while (ii < it_num_cor):
        xx_cor_next = xx_cor
        yy_cor_next = yy_cor + (di * (xx_cor >> ii))
        zz_cor_next = zz_cor - (di * 2**(-ii))
        if (zz_cor_next < 0):
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

    x_val_list = xx_cor_list
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