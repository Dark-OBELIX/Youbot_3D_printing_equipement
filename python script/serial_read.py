import serial
import time
from threading import Thread
from colorama import Fore, Style, init

# Initialiser colorama
init()

# Configuration du port série
arduino_port = 'COM3'  # Remplacez par le port de votre Arduino
baud_rate = 9600

# Initialiser la connexion série
ser = serial.Serial(arduino_port, baud_rate)
time.sleep(2)

def read_from_arduino():
    """Lit en continu la température depuis l'Arduino et l'affiche."""
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(f"Ifo reçue par Python : {Fore.RED}{line}{Style.RESET_ALL}")

def send_target_temperature():
    """Demande et envoie une nouvelle température cible à l'Arduino."""
    while True:
        target_temperature = input("Entrez une nouvelle température cible (ou appuyez sur Entrée pour ignorer) : ")
        if target_temperature:
            # Affiche la température cible en vert
            print(f"Température cible envoyée : {Fore.GREEN}{target_temperature}{Style.RESET_ALL}")
            ser.write(f"{target_temperature}\n".encode())  # Envoie la température cible à l'Arduino
# 
# Crée un thread pour lire les données en continu
thread = Thread(target=read_from_arduino)
thread.daemon = True  # Le thread s'arrête quand le programme principal est arrêté
thread.start()

ser.write(f"S".encode())  # Envoie la température cible à l'Arduino

try:
    send_target_temperature()  # Permet à l'utilisateur de définir la température cible à tout moment
except KeyboardInterrupt:
    print("Arrêt du programme.")
finally:
    ser.close()  # Fermer le port série
