#!/usr/bin/python

# Conversor de archivos de texto del GPS/Acelerometro

# Genera un archivo con los datos del GPS y otro con los datos del acelerometro
# a partir de un archivo .TXT

import sys
import os

total = len(sys.argv)
cmdargs = str(sys.argv)

#print ("The total numbers of args passed to the script: %d " % total)
#print ("Args list: %s " % cmdargs)
# Pharsing args one by one
#print ("Script name: %s" % str(sys.argv[0]))
print ("\nProcesando el archivo: %s\n" % str(sys.argv[1]))
#print ("Second argument: %s" % str(sys.argv[2]))

archi = str(sys.argv[1])
nombre = archi[:-4]
#print(nombre + '.TXT')
txt = open(nombre + '.TXT', 'r')
gps = open('GPS-' + nombre + '.nmea', 'w')
imuace = open('IMUaceleracion-' + nombre + '.txt', 'w')
imudir = open('IMUdirecciones-' + nombre + '.txt', 'w')

lineasGPS = 0
lineasIMU = 0

for line in txt:
    # Diferencia datos del gps y de la imu
    if line[0] == '$':
        gps.write(line)
        lineasGPS += 1
    else:
        # Diferencia direcciones de aceleraciones
        if line.count(',') < 3:
            imuace.write(line)
            lineasIMU += 1
        else:
            imudir.write(line)

txt.close()
gps.close()
imuace.close()
imudir.close()

# Borra el archivo de GPS si esta vacio
if lineasGPS == 0:
    os.remove('GPS-' + nombre + '.nmea')
