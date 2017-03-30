import serial
import time
from math import pi, sqrt, pow, cos, sin, atan2, radians

 
# ------------------VARIABLES---------------------

# Pose actual del robot
X = 0
Y = 0
Theta = 0

# ------------------- CONSTANTES -----------------

VELOCIDAD = 120
SEPARACION_RUEDAS = 121.5
MINIMO_PUNTOS = 10
SEPARACION_BORDE = 400

# ------------------

def envia(missatge,temps):
	ser.write(missatge+'\r'+'\n')
	rbuffer = []
	resp = ''
	time.sleep(temps)
	while ser.inWaiting()>0:
		resp = ser.readline()
		rbuffer.append(resp.split(','))
	return rbuffer
  
ser=serial.Serial(port='/dev/ttyACM0',baudrate=115200,timeout=1)

# --------------------------------------------------

# IN:	(Xfi, Yfi)		Punto al que el robot se dirigira
#
# OUT:	-				El robot se mueve a la posicion dada
def go_to_point(Xfi, Yfi):
	global X, Y, Theta

	# Distancia a la que se encuentra al punto
	Xdist = Xfi - X
	Ydist = Yfi - Y
	dist = sqrt(pow(Xdist, 2) + pow(Ydist, 2))

	steering =  atan2(Ydist, Xdist)
	# Cuanto tendran que recorrer las ruedas para lograr la orientacion adecuada
	movRuedas = SEPARACION_RUEDAS * (steering - Theta)

	# Si hay que modificar la orientacion 
	if (steering - Theta) != 0:
		msg = 'SetMotor LWheelDist ' + str(-movRuedas) + ' RWheelDist ' + str(movRuedas) + ' Speed ' + str(VELOCIDAD)
		envia(msg, 3)
	
	# Nos movemos hacia el punto, la distancia que los separa
	msg = 'SetMotor LWheelDist ' + str(dist) + ' RWheelDist ' + str(dist) + ' Speed ' + str(VELOCIDAD)
	envia(msg, 10)
	
	# Actualizamos las variables globales de la pose del robot
	X = Xfi
	Y = Yfi
	Theta = steering

	
	
# --------------------------------------------------

# IN:	(x1, y1)	Primer punto
#		(x2, y2)	Segundo punto
#
# OUT:				Distancia entre los dos puntos
def dist(x1, y1, x2, y2):
	return sqrt( pow(x2-x1, 2) + pow(y2-y1, 2) )

	

# IN:	(x1, y1)	Primer punto
#		(x2, y2)	Segundo punto
#
# OUT: 	(m)			Pendiente de la recta 
#		(c)			Intercepto de la recta
def calcula_recta (x1,y1, x2,y2):
	m = (y2-y1)/(x2-x1)
	c = -m + y1
	
	return [m, c]	

	
	
# IN: 	(m)			Pendiente de la recta
#		(c)			Intercepto de la recta
#		(punt)		Punto por el que cortara la nueva recta
#		
# OUT:	(m2)		Pendiente de la recta perpendicular a la de entrada
#		(c2)		Intercepto de la recta perpendicular a la de entrada
def calcula_recta_perp(m, c, punt):
	m2 = -1/m						# Pendiente
	c2 = -m2*punt[0] + punt[1]		# Intercepto

	return [m2, c2]

	
# IN:	(recta1)	Pendiente e intercepto de la primera recta
#		(recta2)	Pendiente e intercepto de la segunda recta
#
# OUT:	(x, y)		Punto por el que cortan las dos rectas
def calcula_punto_inter(recta1, recta2):
	x = (recta2[1] - recta1[1])/(recta1[0] - recta2[0])
	y = recta1[0] * x + recta1[1]
	
	return [x, y]
	
	

# IN:	(recta)		Pendiente e intercepto de la recta
#		(punto)		Punto (x, y)
#
#		(True | False)	Si el punto se encuentra a la izq de la recta devuelve True
def ubic_punto_recta(recta, punto):
	return punto[1] > recta[0] * punto[0] + recta[1]
	
	
	
# IN:	(val)			Valor de la distancia obtenida del obstaculo
#
# OUT:	(True | False)	Si es un valor valido de lectura del laser
def valor_valido(val):
	if val > 200 and val < 4000: 
		return True
	return False
	
	

# IN:	(angLaser)		Angulo en el que se ha detectado un obstaculo
#	 	(distLaser)		Distancia a dicho obstaculo
#
# OUT:	(puntoMundo)	Coordenadas del obstaculo detectado en el eje de coordenadas del mundo	  
def robot_to_world_coordinates(angLaser, distLaser):
	# Coordenadas polares a cartesianas del obstaculo
	radiansLaser = radians(angLaser)
	puntoRobot = [distLaser*cos(radiansLaser), distLaser*sin(radiansLaser)]
	
	# Cambio de coordenadas aplicando la matriz de cambio de ejes 
	puntoMundoX = puntoRobot[0]*cos(Theta) + puntoRobot[1]*sin(Theta)
	puntoMundoY = puntoRobot[0]*-sin(Theta) + puntoRobot[1]*cos(Theta)

	# Anadir a las coordenadas la posicion del robot
	return [puntoMundoX + X, puntoMundoY + Y]

	

# IN:	(A, B, C)	Tre puntos
#
# OUT:				Si la pendiente de la recta AB es menor que la pendiente de la recta AC, 
#					entonces los tres puntos se enumeran en el sentido contrario a las agujas del reloj.
def ccw(A,B,C):
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

	
	
# IN: 	(A, B, C, D)	Cuatro puntos
#
# OUT:	Calcula si el segmento AB intersecta o no con el segmento CD
def intersectan(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)
	
	
	
# IN:	(sensor_info)	Informacion que proporciona el laser sobre los obstaculos que detecta
#
# OUT:	(bordes)		Retorna los bordes de los obstaculos que ha detectado el laser
def borde_obstaculos(sensor_info):
	bordes = []
	contador = 0
	cnt_error = 0
	
	borde_ini = []
	borde_final = []
	sensor_info = sensor_info[315:-1] + sensor_info[0:45]

	for fila in sensor_info:
		if contador == 0 and valor_valido(fila[1]):
			borde_ini =  robot_to_world_coordinates(fila[0], fila[1])
			contador += 1
		elif contador > 0 and valor_valido(fila[1]):
			borde_final = robot_to_world_coordinates(fila[0], fila[1])
			contador += 1
			cnt_error = 0
		elif contador > 0:
			cnt_error += 1
			contador += 1
			if cnt_error > 1:
				if contador >= MINIMO_PUNTOS:
					bordes.append(borde_ini)
					bordes.append(borde_final)
				
				contador = 0
				cnt_error = 0
				borde_ini = []
				borde_final = []
				
	return bordes
	
	
	
# IN:	(bordes)		Lista de los bordes de todos los obstacus detectados
#		(Xfi, Yfi)		Punto final
#
# OUT:	(bordes_fin)	Lista con solo los bordes de los obstaculos que se encuentren
#						entre el robot y el punto de destino
def calcula_intersec_segmentos(bordes, Xfi, Yfi):
	bordes_fin = []
	
	for i in range(0, len(bordes), 2):
		# Comprobamos si los segmentos formados por los grupos de bordes, intersectan
		if intersectan(bordes[i], bordes[i+1], [X,Y], [Xfi, Yfi]):
			bordes_fin.append(bordes[i])
			bordes_fin.append(bordes[i+1])
	
	return bordes_fin

	

# IN:	(bordes_n)		Lista de bordes de obstaculos
#		(Xfi, Yfi)		Coordenadas del punto de destino
#
# OUT:	(punt_min_heu)	Borde que este a menor distancia entre la posicion actual
#						del robot y el destino final
def min_heuristic_dist(bordes_n, Xfi, Yfi):
	punt_min_heu = bordes_n[0]
	d_min = dist(X,Y, bordes_n[0][0], bordes_n[0][1])
	
	for i, punt in enumerate(bordes_n, start = 1):
		dist_punt = dist(X, Y, punt[0], punt[1]) + dist(punt[0], punt[1], Xfi, Yfi)
		if dist_punt < d_min:
			punt_min_heu = punt
			d_min = dist_punt
	
	return punt_min_heu


	
# IN:	(Xfi, Yfi)		Punto de destino
#
# OUT:	(-)				El robot se mueve a dicha posicion evitando obstaculos
def go_to_point_obstacles(Xfi, Yfi):
	global Theta
	
	while True:
		# SCAN
		envia('SetLDSRotation On',0.2)
		time.sleep(3) 
		sensor_info = envia('GetLDSScan',0.2)
		sensor_info = sensor_info[2:]
		sensor_info = filter((lambda x: len(x) == 4), sensor_info)

		for i, fila in enumerate(sensor_info):
			sensor_info[i][0] = int(sensor_info[i][0])
			sensor_info[i][1] = int(sensor_info[i][1])
			del fila[2:4]

		envia('SetLDSRotation Off',0.2)
		
		# --------

		# Orientamos el robot hacia el punto de destino
		steering =  atan2(Yfi - Y, Xfi - X)
		movRuedas = SEPARACION_RUEDAS * (steering - Theta)

		if (steering - Theta) != 0:
			msg = 'SetMotor LWheelDist ' + str(-movRuedas) + ' RWheelDist ' + str(movRuedas) + ' Speed ' + str(VELOCIDAD)
			envia(msg, 3)
			Theta = steering

		# Calculamos los bordes de los obstaculos que ve el robot
		bordes_obs = borde_obstaculos(sensor_info)

		if len(bordes_obs) != 0:
			# Contiene solo los bordes de los obstaculos que se encuentran entre
			# el robot y el punto de destino
			bordes_obs_destino = calcula_intersec_segmentos(bordes_obs, Xfi, Yfi)
			
			if len(bordes_obs_destino) != 0:
				# Siguiente punto al que dirigirse
				next_punt = min_heuristic_dist(bordes_obs_destino, Xfi, Yfi)
				
				# Recta entre la posicion actual del robot y el siguiente punto
				recta_a_borde = calcula_recta(X, Y, next_punt[0], next_punt[1])
				
				# Recta perpendicular a la recta entre el robot y el siguiente punto
				recta_perp_borde = calcula_recta_perp(recta_a_borde[0], recta_a_borde[1], next_punt)
				
				# Separacion necesaria para que el robot no colisione con el objeto				
				nueva_recta = recta_a_borde
				
				esta_derecha =  ubic_punto_recta(recta_a_borde, next_punt)
				if esta_derecha:
					nueva_recta[1] = nueva_recta[1] + SEPARACION_BORDE
				else:
					nueva_recta[1] = nueva_recta[1] - SEPARACION_BORDE
				
				
				# Punto al que se dirigira el robot
				nuevo_punto_des = calcula_punto_inter(recta_perp_borde, nueva_recta)
				
				go_to_point(nuevo_punto_des[0], nuevo_punto_des[1])	
															

			else:
				go_to_point(Xfi, Yfi)
				break
		else:
			go_to_point(Xfi, Yfi)
			break
		
	

def main():

	go_to_point_obstacles(2500, 0)
	

if __name__ == "__main__": main()






