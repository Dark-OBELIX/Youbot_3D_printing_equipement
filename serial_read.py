import serial
import time
from colorama import Fore, Style, init

# Initialiser colorama
init()

# Configuration du port série
arduino_port = 'COM3'  # CMD
baud_rate = 9600

# Initialiser la connexion série
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            # Affiche le texte en couleur normale, et le chiffre en rouge
            print(f"Température reçue par python : {Fore.RED}{line}{Style.RESET_ALL}")
except KeyboardInterrupt:
    print("Arrêt du programme.")
finally:
    ser.close()  # Ferme le port série
