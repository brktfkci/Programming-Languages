import math

it_num = 15   ## iteration number
uu = 1        ## for circular, uu = 1; for linear, uu = 0; for hyperbolic, uu = -1;
mode = 0      ## for rotation, mode = 0; for vectoring, mode = 1;

xx = 10
yy = 50
theta = 0     ## give radian value



if (mode == 0):
	if (theta < 0):
		dd = -1
	elif (theta == 0):
		dd = 0
	elif (theta > 0):
		dd = 1
elif (mode == 1):
	if (int(xx*yy) < 0):
		dd = 1
	elif (int(xx*yy) == 0):
		dd = 0
	elif (int(xx*yy) > 0):
		dd = -1
		

		
def function_cordic (xx_cor, yy_cor, theta_cor, uu_cor, dd_cor, it_num_cor):
	list_degree_circ = [32768, 19344, 10221, 5188, 2604, 1303, 652, 326, 163, 81, 41, 20, 10, 5, 3, 1]
	# list_degree_hypr = [32768, 19344, 10221, 5188, 2604, 1303, 652, 326, 163, 81, 41, 20, 10, 5, 3, 1]
    Ki = Ki_calc(it_num_cor)
    theta_curr = int(32768 * 4 * (theta_req / math.pi))
    
	xx_cor_list = []
	yy_cor_list = []
	theta_cor_list = []
    
	if (uu_cor == 1):
		psu_theta_cor = 45
	elif (uu_cor == 0):
		psu_theta_cor = 1
	elif (uu_cor == -1):
		psu_theta_cor = 0

	ii = 0
	while (ii < it_num_cor):
		xx_cor_next = xx_cor - (uu_cor * dd_cor * yy_cor * (2 ** (-ii)))
		yy_cor_next = yy_cor + (dd_cor * xx_cor * (2 ** (-ii)))
		theta_cor_next = theta_cor - (dd_cor * psu_theta_cor)
		if (uu_cor == 1):
			psu_theta_cor_next = math.atan(deg2rad(2**(-ii)))
		elif (uu_cor == 0):
			psu_theta_cor_next = 2 ** (-ii)
		elif (uu_cor == -1):
			psu_theta_cor_next = math.atanh(deg2rad(2**(-ii)))
		
		xx_cor = xx_cor_next
		yy_cor = yy_cor_next
		theta_cor = theta_cor_next
		
		xx_cor_list.append(xx_cor)
		yy_cor_list.append(yy_cor)
		theta_cor_list.append(theta_cor)
		
		ii = ii + 1

	return xx_cor_list, yy_cor_list, theta_cor_list
	
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


# xx_list, yy_list, theta_list = function_cordic(xx, yy, theta, uu, dd, it_num)	

# print(xx_list)
# print(yy_list)
# print(theta_list)

