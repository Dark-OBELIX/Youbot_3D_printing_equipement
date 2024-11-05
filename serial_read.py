import serial
import time
from colorama import Fore, Style, init

# Initialiser colorama
init()

# Configuration du port série
arduino_port = 'COM3'  # Remplacez par le port de votre Arduino
baud_rate = 9600

# Initialiser la connexion série
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)

try:
    # Demander la température cible à l'utilisateur
    target_temperature = input("Entrez la température cible : ")
    ser.write(f"{target_temperature}\n".encode())  # Envoie la température cible à l'Arduino

    while True:
        if ser.in_waiting > 0:
            # Lire et afficher la température actuelle reçue de l'Arduino
            line = ser.readline().decode('utf-8').rstrip()
            print(f"Température reçue par python : {Fore.RED}{line}{Style.RESET_ALL}")

except KeyboardInterrupt:
    print("Arrêt du programme.")
finally:
    ser.close()  # Fermer le port série
