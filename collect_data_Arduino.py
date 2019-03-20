import serial
import time
import json
import csv
import sys

ser = serial.Serial('COM9', 115200, timeout=0)
with open('lecturas_MPU.csv', 'a', newline='') as csvfile: 
	regwriter = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
	while True:
		try:
			data1 = ser.readline()
			data = data1.decode('utf8').replace("'",'"')
			#print(data)
			HUD_data = json.loads(data)
			# Obteniendo la lectura 
			a_x = HUD_data['ax']
			a_y = HUD_data['ay']
			a_z = HUD_data['az']
			g_x = HUD_data['gx']
			g_y = HUD_data['gy']
			g_z = HUD_data['gz']
			target = HUD_data['tg']
			print (" acc_x ={} acc_y={} acc_z ={} Target={}".format(a_x,a_y,a_z,target))

			regwriter.writerow([str(a_x), str(a_y),str(a_z),str(g_x), str(g_y), str(g_z),str(target)])
			#print(data1)
			#time.sleep(0.01)
		except:
			#print("no reads")
			time.sleep(0.1)
			pass
