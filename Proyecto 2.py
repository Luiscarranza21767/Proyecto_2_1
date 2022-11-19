# Import standard python modules
import time
import sys
import serial

from Adafruit_IO import MQTTClient

# Import Adafruit IO REST client.
from Adafruit_IO import Client, Feed

ADAFRUIT_IO_USERNAME = "LuisCarranza21767"
ADAFRUIT_IO_KEY = "aio_pFAt390p9tYPfbVTPYAJCdoGeu5E"

client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

FEED_ID1 = 'Movimiento_1'
FEED_ID2 = 'Movimiento_2'
FEED_ID3 = 'Movimiento_3'
FEED_ID4 = 'Cierre_pinza'
FEED_ID5 = 'Modo'
FEED_ID6 = 'Guardar'
FEED_ID7 = 'Posicion'
FEED_ID8 = 'Conectado'

decenas = 0
centenas = 0
unidades = 0

PIC = serial.Serial(port = 'COM6', baudrate = 9600, timeout= 1)

def sep(num):
    global centenas
    global decenas
    global unidades
    centenas = int(num/100)
    res = num % 100
    decenas = int(res/10)
    unidades = res%10
    
# Define callback functions which will be called when certain events happen.
def connected(client):
    # Subscribe to changes on a feed named Counter.
    print('Subscribing to Feed {0}'.format(FEED_ID1))
    client.subscribe(FEED_ID1)
    print('Subscribing to Feed {0}'.format(FEED_ID2))
    client.subscribe(FEED_ID2)
    print('Subscribing to Feed {0}'.format(FEED_ID3))
    client.subscribe(FEED_ID3)
    print('Subscribing to Feed {0}'.format(FEED_ID4))
    client.subscribe(FEED_ID4)
    print('Subscribing to Feed {0}'.format(FEED_ID5))
    client.subscribe(FEED_ID5)
    print('Subscribing to Feed {0}'.format(FEED_ID6))
    client.subscribe(FEED_ID6)
    print('Subscribing to Feed {0}'.format(FEED_ID7))
    client.subscribe(FEED_ID7)
    print('Subscribing to Feed {0}'.format(FEED_ID8))
    client.subscribe(FEED_ID8)
    print('Waiting for feed data...')

def disconnected(client):
    """Disconnected function will be called when the client disconnects."""
    sys.exit(1)

def message(client, feed_id, payload):
    

    print('Feed {0} received new value: {1}'.format(feed_id, payload))
#     valor = int(payload, 10)
#     valor1 = bin(valor)
#     PIC.write(bytes(payload, 'utf-8'))
#     time.sleep(0.05)
    
    if(feed_id == 'Modo'):
        PIC.write(bytes('a', 'utf-8'))
        time.sleep(0.05)
        PIC.write(bytes(payload, 'utf-8'))
        time.sleep(0.05)
        print('Modo recibió:', payload)
            
    if(feed_id == 'Guardar'):
        PIC.write(bytes('b', 'utf-8'))
        time.sleep(0.05)
        PIC.write(bytes(payload, 'utf-8'))
        time.sleep(0.05)
        print('Guardar recibió:', payload)
            
    if(feed_id == 'Posicion'):
        PIC.write(bytes('c', 'utf-8'))
        time.sleep(0.03)
        PIC.write(bytes(payload, 'utf-8'))
        time.sleep(0.03)
        print('Nueva posición:', payload)
            
    if(feed_id == 'Movimiento_1' or feed_id == 'Movimiento_2' or feed_id == 'Movimiento_3' or feed_id == 'Cierre_pinza'):
        if (feed_id == 'Movimiento_1'):
            PIC.write(bytes('d', 'utf-8'))
        elif (feed_id == 'Movimiento_2'):
            PIC.write(bytes('e', 'utf-8'))
        elif (feed_id == 'Movimiento_3'):
            PIC.write(bytes('f', 'utf-8'))
        elif (feed_id == 'Cierre_pinza'):
            PIC.write(bytes('g', 'utf-8'))
            
        time.sleep(0.05)
        
        numero = int(payload, 10)
        sep(numero)
        cen = str(centenas)
        dec = str(decenas)
        uni = str(unidades)
        PIC.write(bytes(cen, 'utf-8'))
        time.sleep(0.05)
        PIC.write(bytes(dec, 'utf-8'))
        time.sleep(0.05)
        PIC.write(bytes(uni, 'utf-8'))
        time.sleep(0.05)
        print('el valor es: ')
        print(cen+dec+uni)
        
#         print('Las centenas son: ', centenas)
#         print('Las decenas son: ', decenas)
#         print('Las unidades son: ', unidades)
        
#         print('Se está controlando: movimiento 1')
        
#        if(feed_id == 'Movimiento_2'):
#             PIC.write(bytes(e, 'utf-8'))
#             time.sleep(0.05)
#            print('Se está controlando: movimiento 2')
#        if(feed_id == 'Movimiento_3'):
#             PIC.write(bytes(f, 'utf-8'))
#             time.sleep(0.05)
#            print('Se está controlando: movimiento 3')
#        if(feed_id == 'Cierre_pinza'):
#             PIC.write(bytes(g, 'utf-8'))
#             time.sleep(0.05)
#            print('Se está controlando: la garra')
            
    


# Setup the callback functions defined above.
client.on_connect = connected
client.on_disconnect = disconnected
client.on_message = message

# Connect to the Adafruit IO server.
client.connect()

# The first option is to run a thread in the background so you can continue
# doing things in your program.

client.loop_blocking()
