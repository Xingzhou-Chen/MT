# Bibliotheken laden
from machine import UART, Pin
from time import sleep

# UART0 initialisieren
uart0 = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1), bits=8, parity=None, stop=1)
print('UART0:', uart0)
print()

# UART1 initialisieren
uart1 = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9), bits=8, parity=None, stop=1)
print('UART1:', uart1)
print()

# Daten zum Senden
txData = 'Hallo Welt'
print('Daten senden:', txData)

# Daten senden
i=0
while True:
#     uart0.write(txData)
    i=i+1
    uart0.write(str(i)+'\n')
    sleep(0.1)

# Daten empfangen und ausgeben
# rxData = uart1.readline()
# print('Daten empfangen:', rxData.decode('utf-8'))