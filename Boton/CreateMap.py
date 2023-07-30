import folium
import webbrowser
import os
import pandas as pd


#Comentar estas dos lineas iniciales is se desea abrir el archivo en el navegador por defecto
browser_path="<:\Program Files\BraveSoftware\Brave-Browser\Application\brave.exe>" #Path of your browser
webbrowser.register('Brave', None, webbrowser.BackgroundBrowser(browser_path))  # Chrome, Firefox, etc.

map_center = [6.263432, -75.561852]  # Home, centrar el mapa donde está el gateway
my_map = folium.Map(location=map_center, zoom_start=20)

def decode(input_string):
    rssi_hex = input_string[:8]  # First 8 characters for RSSI in hexadecimal
    snr_hex = input_string[8:16]  # Next 8 characters for SNR in hexadecimal
    message_hex = input_string[16:]  # Rest of the characters for the message in hexadecimal

    # Convert hexadecimal strings to integers
    rssi = int(rssi_hex, 16) - 0x100000000
    snr = int(snr_hex, 16)/10

    # Convert the remaining hexadecimal part to ASCII to get the message
    message_ascii = bytes.fromhex(message_hex).decode('utf-8')

    return rssi, snr, message_ascii

#function that read a archive txt and take every line as string and print it

def read_txt(dir):
    f = open(dir, 'r')
    for line in f:
        print(line)
        RSSI, SNR, msn = decode(str(line))
        telemetria = msn.split(',')
        folium.Marker(location=(telemetria[1],telemetria[2]),icon=folium.Icon(color='red', icon='ok-circle', prefix='fa')).add_to(my_map)
        my_map.save('recorrido.html')
    f.close()

name_text = 'output.txt'    
read_txt(name_text)
webbrowser.open('file://' + os.path.realpath('recorrido.html'))