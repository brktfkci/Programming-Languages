########################################################
#####        CORDIC Linear Vectoring Mode        #######
########################################################
##### xx initial = x     >>>  xn = xx              #####
##### yy initial = y     >>>  yn = 0               #####
##### zz initial = 0     >>>  zn = z + yy/xx       #####
########################################################
import math

it_num = 15  ## iteration number
xx = 10000    ## 
yy = 200     ## 
theta = 0    ## give radian value
	
def function_cordic (xx_cor, yy_cor, zz_cor, it_num_cor):
    list_degree_lin = [32768, 16384, 8192, 4096, 2048, 1024, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1]
    # Ki = list_degree_lin[0]
    if()

    xx_cor_list = []
    yy_cor_list = []
    zz_cor_list = []
    di = -1

    ii = 0
    while (ii < it_num_cor):
        xx_cor_next = xx_cor
        yy_cor_next = yy_cor + (di * (xx_cor >> ii))
        zz_cor_next = zz_cor - (di * list_degree_lin[ii])
        if (yy_cor_next < 0):
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

    x_val_list = xx_cor_list
    y_val_list = yy_cor_list
    # z_val_list = [(element / Ki) for element in zz_cor_list]
    z_val_list = zz_cor_list

    return x_val_list, y_val_list, z_val_list
	
def Ki_calc (num_iter):
	result_calc = 1
	for ii in range (num_iter):
		result_calc = result_calc * (math.sqrt(1+(2**(-2*ii))))
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
list_xxx = []



