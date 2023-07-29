import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import pandas as pd
import time

# Set up a TCP/IP server
tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
 
# Bind the socket to server address and port 81
server_address = ('192.168.28.142', 80)
tcp_socket.bind(server_address)
 
# Listen on port 81
tcp_socket.listen(1)
datos = pd.DataFrame(columns=['AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ', 'MagX', 'MagY', 'MagZ', 'PWM1', 'PWM2', 'PWM3', 'PWM4', 'Latitude', 'Longitude'])
datos.to_csv('datos.csv', index=False)

accx = [[0], [0]]
accy = [[0], [0]]
accz = [[0], [0]]
gyrox = [[0], [0]]
gyroy = [[0], [0]]
gyroz = [[0], [0]]
magx = [[0], [0]]
magy = [[0], [0]]
magz = [[0], [0]]
pwm1 = [[0], [0]]
pwm2 = [[0], [0]]
pwm3 = [[0], [0]]
pwm4 = [[0], [0]]
#Configuramos la gráfica
fig, ax= plt.subplots(3,5)
fig.set_size_inches(15, 6)
fig.tight_layout(pad=3.0)
hl, = ax[0, 0].plot(accx[0], accx[1], 'r')
h2, = ax[1, 0].plot(accy[0], accy[1], 'b')
h3, = ax[2, 0].plot(accz[0], accz[1], 'g')
h4, = ax[0, 1].plot(gyrox[0], gyrox[1], 'r')
h5, = ax[1, 1].plot(gyroy[0], gyroy[1], 'b')
h6, = ax[2, 1].plot(gyroz[0], gyroz[1], 'g')
h7, = ax[0, 2].plot(magx[0], magx[1], 'r')
h8, = ax[1, 2].plot(magy[0], magy[1], 'b')
h9, = ax[2, 2].plot(magz[0], magz[1], 'g')
h10, = ax[0, 3].plot(pwm1[0], pwm1[1], 'r')
h11, = ax[1, 3].plot(pwm2[0], pwm2[1], 'b')
h12, = ax[2, 3].plot(pwm3[0], pwm3[1], 'g')
h13, = ax[0, 4].plot(pwm4[0], pwm4[1], 'r')

#limits in axis
ax[0, 0].set(xlim=(0, 50), ylim=(-2, 2), xlabel='Time (s)', ylabel='[g]', title='AccX')
ax[1, 0].set(xlim=(0, 50), ylim=(-2, 2), xlabel='Time (s)', ylabel='[g]', title='AccY')
ax[2, 0].set(xlim=(0, 50), ylim=(-2, 2), xlabel='Time (s)', ylabel='[g]', title='AccZ')
ax[0, 1].set(xlim=(0, 50), ylim=(-250, 250), xlabel='Time (s)', ylabel='[grad/s]', title='GyroX')
ax[1, 1].set(xlim=(0, 50), ylim=(-250, 250), xlabel='Time (s)', ylabel='[grad/s]', title='GyroY')
ax[2, 1].set(xlim=(0, 50), ylim=(-250, 250), xlabel='Time (s)', ylabel='[grad/s]', title='GyroZ')
ax[0, 2].set(xlim=(0, 50), ylim=(-4800, 4800), xlabel='Time (s)', ylabel='[μT', title='MagX')
ax[1, 2].set(xlim=(0, 50), ylim=(-4800, 4800), xlabel='Time (s)', ylabel='[μT]', title='MagY')
ax[2, 2].set(xlim=(0, 50), ylim=(-4800, 4800), xlabel='Time (s)', ylabel='[μT]', title='MagZ')
ax[0, 3].set(xlim=(0, 50), ylim=(-1, 100), xlabel='Time (s)', ylabel='[%]', title='PWM Motor 1')
ax[1, 3].set(xlim=(0, 50), ylim=(-1, 100), xlabel='Time (s)', ylabel='[%]', title='PWM Motor 2')
ax[2, 3].set(xlim=(0, 50), ylim=(-1, 100), xlabel='Time (s)', ylabel='[%]', title='PWM Motor 3')
ax[0, 4].set(xlim=(0, 50), ylim=(-1, 100), xlabel='Time (s)', ylabel='[%]', title='PWM Motor 4')

def GetData(out_data):
    while True:
        print("Waiting for connection")
        connection, client = tcp_socket.accept()
        
        try:
            print("Connected to client IP: {}".format(client))
            
            # Receive and print data 64 bytes at a time, as long as the client is sending something
            while True:
                data = connection.recv(256)
                if (str(data)!="b''"):
                    data_str = str(data).removeprefix("b'").removesuffix("'")
                    telemetria = data_str.split(',')
                    accx[1].append(float((telemetria[0])))
                    accy[1].append(float((telemetria[1])))
                    accz[1].append(float((telemetria[2])))
                    gyrox[1].append(float((telemetria[3])))
                    gyroy[1].append(float((telemetria[4])))
                    gyroz[1].append(float((telemetria[5])))
                    magx[1].append(float((telemetria[6])))
                    magy[1].append(float((telemetria[7])))
                    magz[1].append(float((telemetria[8])))
                    pwm1[1].append(float((telemetria[9])))
                    pwm2[1].append(float((telemetria[10])))
                    pwm3[1].append(float((telemetria[11])))
                    pwm4[1].append(float((telemetria[12])))
                    df = pd.read_csv('datos.csv')
                    new_data = {'AccX': float(telemetria[0]), 'AccY': float(telemetria[1]), 'AccZ': float(telemetria[2]), 
                                'GyroX': float(telemetria[3]), 'GyroY': float(telemetria[4]), 'GyroZ': float(telemetria[5]), 
                                'MagX': float(telemetria[6]), 'MagY': float(telemetria[7]), 'MagZ': float(telemetria[8]), 
                                'PWM1': float(telemetria[9]), 'PWM2': float(telemetria[10]), 'PWM3': float(telemetria[11]), 
                                'PWM4': float(telemetria[12]), 'Latitude': float(telemetria[13]), 'Longitude': float(telemetria[14])}
                    df = df._append(new_data, ignore_index=True)
                    df.to_csv('datos.csv', index=False)
                    
                    if(len(accx[1]) > 50):
                        accx[1].pop(0)
                        accy[1].pop(0)
                        accz[1].pop(0)
                        gyrox[1].pop(0)
                        gyroy[1].pop(0)
                        gyroz[1].pop(0)
                        magx[1].pop(0)
                        magy[1].pop(0)
                        magz[1].pop(0)
                        pwm1[1].pop(0)
                        pwm2[1].pop(0)
                        pwm3[1].pop(0)
                        pwm4[1].pop(0)
                        
                    print("Received data: {}".format(data))
                    
                if (not data or TimeoutError):
                    break
        finally:
            connection.close()
        
def update_line(num, hl, data):
    hl.set_data(range(len(data[1])), data[1])
    return hl,
# Configuramos la función que "animará" nuestra gráfica
line_ani = animation.FuncAnimation(fig, update_line, fargs=(hl, accx), interval=50, blit=False, save_count=50)
line_ani2 = animation.FuncAnimation(fig, update_line, fargs=(h2, accy), interval=50, blit=False, save_count=50)
line_ani3 = animation.FuncAnimation(fig, update_line, fargs=(h3, accz), interval=50, blit=False, save_count=50)
line_ani4 = animation.FuncAnimation(fig, update_line, fargs=(h4, gyrox), interval=50, blit=False, save_count=50)
line_ani5 = animation.FuncAnimation(fig, update_line, fargs=(h5, gyroy), interval=50, blit=False, save_count=50)
line_ani6 = animation.FuncAnimation(fig, update_line, fargs=(h6, gyroz), interval=50, blit=False, save_count=50)
line_ani7 = animation.FuncAnimation(fig, update_line, fargs=(h7, magx), interval=50, blit=False, save_count=50)
line_ani8 = animation.FuncAnimation(fig, update_line, fargs=(h8, magy), interval=50, blit=False, save_count=50)
line_ani9 = animation.FuncAnimation(fig, update_line, fargs=(h9, magz), interval=50, blit=False, save_count=50)
line_ani10 = animation.FuncAnimation(fig, update_line, fargs=(h10, pwm1), interval=50, blit=False, save_count=50)
line_ani11 = animation.FuncAnimation(fig, update_line, fargs=(h11, pwm2), interval=50, blit=False, save_count=50)
line_ani12 = animation.FuncAnimation(fig, update_line, fargs=(h12, pwm3), interval=50, blit=False, save_count=50)
line_ani13 = animation.FuncAnimation(fig, update_line, fargs=(h13, pwm4), interval=50, blit=False, save_count=50)

# Configuramos y lanzamos el hilo encargado de leer datos del serial
dataCollector = threading.Thread(target = GetData, args=(accx,))
dataCollector.start()
plt.show()
dataCollector.join()
