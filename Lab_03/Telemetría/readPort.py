import socket
import sys
import time
import csv

def create_socket():
    # Crea un objeto socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("Socket creado")
        return sock
    except socket.error as err:
        print("Error al crear el socket: {}".format(err))
        sys.exit()

def bind_socket(sock, host, port):
    # Enlaza el socket al host y puerto específico
    try:
        sock.bind((host, port))
        print("Socket enlazado al puerto: {}".format(port))
    except socket.error as err:
        print("Error al enlazar el socket: {}".format(err))
        sys.exit()

def listen_for_connections(sock, backlog):
    # Escucha conexiones entrantes
    try:
        sock.listen(backlog)
        print("Esperando conexiones entrantes...")
    except socket.error as err:
        print("Error al intentar escuchar: {}".format(err))
        sys.exit()

def accept_connection(sock):
    # Acepta la conexión entrante
    conn, address = sock.accept()
    print("Conexión establecida con {}:{}".format(address[0], address[1]))
    return conn

def receive_data(conn, buffer_size=256):
    # Recibe los datos de la conexión
    data = conn.recv(buffer_size)
    data = data.decode()
    print("Datos recibidos: {}".format(data))
    return data

def save_to_csv(data):
    # Guarda los datos recibidos en un archivo CSV
    
    with open('datos.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        data_str = data.strip()  # Convertir los datos a una cadena y eliminar espacios en blanco
        writer.writerow(data_str.split(','))  # Escribir los datos en el archivo CSV
    print("Datos guardados en datos.csv")

def check_connection(host, port):
    # Comprueba la conexión al host y puerto
    while True:
        try:
            sock = create_socket()
            sock.connect((host, port))
            sock.close()
            print("Conexión exitosa a {}:{}".format(host, port))
        except socket.error as err:
            print("Error al conectar a {}:{}. Reintentando en 5 segundos...".format(host, port))
            time.sleep(5)

def main():
    with open('datos.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(("Latitud,Longitud,accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz").split(','))
    host = '192.168.70.109'  # Escucha en todas las interfaces de red disponibles
    port = 8080  # Puerto de escucha
    backlog = 5  # Número máximo de conexiones pendientes en la colas

    sock = create_socket()
    bind_socket(sock, host, port)
    listen_for_connections(sock, backlog)

    while True:
        conn = accept_connection(sock)
        data = receive_data(conn)
        save_to_csv(data)
        conn.close()

if __name__ == '__main__':
    main()
