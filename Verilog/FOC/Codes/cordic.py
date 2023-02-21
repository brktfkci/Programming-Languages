import math

it_num = 15   ## iteration number
uu = 1        ## for circular, uu = 1; for linear, uu = 0; for hyperbolic, uu = -1;
xx = 10
yy = 50
theta = 0

mode = 0      ## for rotation, mode = 0; for vectoring, mode = 1;

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
	list_degree_hypr = [32768, 19344, 10221, 5188, 2604, 1303, 652, 326, 163, 81, 41, 20, 10, 5, 3, 1]
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

def pos_calculation (x_i, y_i, theta_req):
	list_degree = [32768, 19344, 10221, 5188, 2604, 1303, 652, 326, 163, 81, 41, 20, 10, 5, 3, 1]
	Ki = Ki_calc(15)
	print("Ki = ", Ki)

	x_i = int(x_i)
	y_i = int(y_i)
	
	x_val_list = []
	y_val_list = []
	t_val_list = []	

	index = 1
	dir = 0
	theta_curr = int(32768 * 4 * (theta_req / math.pi)) -32768
	theta_init = 0
	xi_next = 0
	yi_next = 0


	while (index < len(list_degree)-1):
		
		x_val_list.append(x_i)
		y_val_list.append(y_i)
		t_val_list.append(theta_curr)


		
##########################################################################

		if (dir == 1):
			xi_next = x_i + (y_i >> (index -1))
			yi_next = y_i - (x_i >> (index -1))
		elif(dir == 0):
			xi_next = x_i - (y_i >> (index -1))
			yi_next = y_i + (x_i >> (index -1))	

			
		x_i = xi_next
		y_i = yi_next
		
		if (theta_curr < theta_init):
			theta_curr = theta_curr + list_degree[index]
			dir = 0;
		else:
			theta_curr = theta_curr - list_degree[index]
			dir = 1
		print(dir)
##########################################################################

		# if (dir == 0):
		# 	xi_next = x_i + (y_i >> (index -1))
		# 	yi_next = y_i - (x_i >> (index -1))
		# elif(dir == 1):
		# 	xi_next = x_i - (y_i >> (index -1))
		# 	yi_next = y_i + (x_i >> (index -1))	

			
		# x_i = xi_next
		# y_i = yi_next
		
		# if (y_i > 0):
		# 	theta_curr = theta_curr + list_degree[index]
		# 	dir = 0;
		# else:
		# 	theta_curr = theta_curr - list_degree[index]
		# 	dir = 1

##########################################################################
	

		index = index + 1

	print(x_val_list)
	print(y_val_list)
	print(t_val_list)
	hex_list = []
	bits = 20
	for ii in range(len(t_val_list)):
		if(t_val_list[ii] < 0 ):
			hex_list.append(((hex((1 << bits) + t_val_list[ii]))[2:]).upper())
		else:
			hex_list.append(('{:{fill}{width}{base}}'.format(t_val_list[ii], fill='0', width='5', base='x')).upper())	

	f=open('data','w')
	for ele in hex_list:
		f.write(str(ele))
		f.write("\n")
	f.close()


	print("-------------")
	x_val_list_sc = [element / Ki for element in x_val_list]
	y_val_list_sc = [element / Ki for element in y_val_list]
	t_val_list_sc = [element / Ki for element in t_val_list]

	return x_val_list_sc, y_val_list_sc, t_val_list_sc

	
# xx_list, yy_list, theta_list = function_cordic(xx, yy, theta, uu, dd, it_num)	

# print(xx_list)
# print(yy_list)
# print(theta_list)

xi = 1024
yi = 0
thetai = (math.pi/3)


# xi = 600
# yi = 800
# thetai = 0

list_x_out, list_y_out, list_theta_out = pos_calculation(xi, yi, thetai)



# print("---------------------------------")
# print(list_x_out)
# print(list_y_out)
# print(list_theta_out)


# print(math.tanh(math.e))