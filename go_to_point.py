import serial
import time
from math import pi, sqrt, pow, atan2
from numpy import sign
 
# ------------------VARIABLES---------------------

# Pose actual del robot
X = 1320
Y = 950
Theta = 0

# ------------------- CONSTANTES -----------------

VELOCIDAD = 180
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

def main():
	go_to_point(3720, 2550)


if __name__ == "__main__": main()


