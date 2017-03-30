import serial
import time
from math import pi, sqrt, pow, cos, sin, radians
 
# ------------------VARIABLES---------------------

# Pose actual del robot
X = 0
Y = 0
Theta = 0

# ------------------- CONSTANTES -----------------

VELOCIDAD = 120
SEPARACION_RUEDAS = 121.5

# ------------------------------------------------

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

#----------------	



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
# OUT:	(puntoRobot)	Coordenadas cartesianas del obstaculo detectado   
def polar_a_cartesiano(angLaser, distLaser):
	# Coordenadas polares a cartesianas del obstaculo
	radiansLaser = radians(angLaser)
	puntoRobot = [distLaser*cos(radiansLaser), distLaser*sin(radiansLaser)]

	return puntoRobot
	
	
	
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


# IN:	(sensor_info)	Informacion que proporciona el laser sobre los obstaculos que detecta
#
# OUT:	(bordes)		Retorna el borde mas largo detectado
def max_obstaculo(sensor_info):
	dist_cerca = 4000
	angulo_cercano = 0

	#buscar punto mas cercano
	for fila in sensor_info:

		if dist_cerca > fila[1] and valor_valido(fila[1]):
			dist_cerca = fila[1]
			angulo_cercano = fila[0]

	angulo_max = angulo_cercano
	angulo_min = angulo_cercano

	contadorTop = 0
	cnt_error = 0
	#buscar direccion a la que ir
	for i in range(angulo_cercano, 360):
		if contadorTop == 0 and valor_valido(sensor_info[i][1]):
			angulo_max = i
			contadorTop += 1
		elif contadorTop > 0 and valor_valido(sensor_info[i][1]):
			if(sensor_info[angulo_max][1] > sensor_info[i][1]):
				break
			angulo_max = i
			contadorTop += 1
			cnt_error = 0
		elif contadorTop > 0:
			cnt_error += 1
			contadorTop += 1
			if cnt_error > 1:
				break

	cnt_error = 0
	contadorBot = 0
	for i in range(angulo_cercano, 0):
		if contadorBot == 0 and valor_valido(sensor_info[i][1]):
			angulo_min = i
			contadorBot += 1
		elif contadorBot > 0 and valor_valido(sensor_info[i][1]):
			if(sensor_info[angulo_min][1] > sensor_info[i][1]):
				break
			angulo_min = i
			contadorBot += 1
			cnt_error = 0
		elif contadorBot > 0:
			cnt_error += 1
			contadorBot += 1
			if cnt_error > 1:
				break

	if(contadorBot < contadorTop):
		return [angulo_cercano, angulo_max]
	return [angulo_cercano, angulo_min]


	
#----------------	


# IN: 	-		
#
# OUT:		Se escanean los alrededores del robot
def scan():
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
	return sensor_info
	

	
	
	
# IN:	(distancia_a_pared)		Distancia a la que el robot se posicionara respecto a la pared
#
# OUT:	-						El robot sigue la pared mas cercana a esa distancia
def follow_a_wall_at_distance(distancia_a_pared):
	sensor_info = scan()
	distancia_maxima = distancia_a_pared + 200
	
	pared = max_obstaculo(sensor_info)


	grados = pared[0]
	if grados > 180:
		grados = grados - 360
	movRuedas = SEPARACION_RUEDAS * radians(grados)

	msg = 'SetMotor LWheelDist ' + str(-movRuedas) + ' RWheelDist ' + str(movRuedas) + ' Speed ' + str(VELOCIDAD)
	envia(msg, 3)

	dist = sensor_info[pared[0]][1] - distancia_a_pared
	msg = 'SetMotor LWheelDist ' + str(dist) + ' RWheelDist ' + str(dist) + ' Speed ' + str(VELOCIDAD)
	envia(msg, 3)

	izquierda = pared[0] > pared[1]
	if izquierda:
		movRuedas = SEPARACION_RUEDAS * radians(-90)
		msg = 'SetMotor LWheelDist ' + str(-movRuedas) + ' RWheelDist ' + str(movRuedas) + ' Speed ' + str(VELOCIDAD)
		envia(msg, 3)
	else:
		movRuedas = SEPARACION_RUEDAS * radians(90)
		msg = 'SetMotor LWheelDist ' + str(-movRuedas) + ' RWheelDist ' + str(movRuedas) + ' Speed ' + str(VELOCIDAD)
		envia(msg, 3)

	msg = 'SetMotor LWheelDist ' + str(500) + ' RWheelDist ' + str(500) + ' Speed ' + str(VELOCIDAD)
	envia(msg, 3)

	while True:
		sensor_info = scan()
		if izquierda:
			sensor_info = sensor_info[90:80]
		else:
			sensor_info = sensor_info[270:280]

		encontrado = False
		for i in sensor_info:

			if i[1] < distancia_maxima and valor_valido(i[1]):

				msg = 'SetMotor LWheelDist ' + str(500) + ' RWheelDist ' + str(500) + ' Speed ' + str(VELOCIDAD)
				envia(msg, 3)
				encontrado = True
				break
		if not encontrado:
			break

			
			

def main():
	follow_a_wall_at_distance(350)

if __name__ == "__main__": main()