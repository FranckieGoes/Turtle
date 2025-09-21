import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)  # adapte le port si besoin

while True:
    ligne = ser.readline().decode().strip()
    print("Valeur reçue :", ligne)