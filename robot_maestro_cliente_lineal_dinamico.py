#!/usr/bin/env python
###################################################################################
# Programa realizado por Carlos Andrés Villalobos Aranda
###################################################################################
# Para iniciar correctamente el programa coloquelo dentro de la carpeta "scripts" #
# que sen encuentra en la carpeta controlador en la siguiente direccion:          #
# ~/husarion_ws/src/controlador/scripts                                           #
###################################################################################
# Importacion de librerias y paqueterias necesarias
import rospy
import math
import sys
import socket
import struct
import matplotlib.pyplot as plt
import scipy.io as sio
from pylab import *
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
###################################################################################
# Declaracion de variables de posicion y orientacion
px = 0 #--------> Condicion inicial de la posicion del robot en x
py = 0 #--------> Condicion inicial de la posicion del robot en y

xi = 0.1 #------> Condicion inicial del estado auxiliar

t = 0 #---------> Tiempo del robot

bat = 0 #-------> Bateria del robot

quatx1 = 0 #----> Cuaternion en x del topico /imu
quaty1 = 0 #----> Cuaternion en y del topico /imu
quatz1 = 0 #----> Cuaternion en z del topico /imu
unitq1 = 0 #----> Cuaternion en x del topico /imu

a11 = 0 #-------> Elemento a11 de la matriz de rotacion en R²
a12 = 0 #-------> Elemento a12 de la matriz de rotacion en R²
a21 = 0 #-------> Elemento a21 de la matriz de rotacion en R²
a22 = 0 #-------> Elemento a22 de la matriz de rotacion en R²

R = eye(2) #----> Condicion inicial de la matriz e rotacion
###################################################################################
# Declaracion de variables y condiciones iniciales para el sistema hipercaotico
A = 2000 #----------------> Amplitud para encriptación de señales

b = -0.5 #----------------> 1er parámetro
c = 4 #-------------------> 2do parámetro
eta = -0.5 #--------------> 3er parámetro

dT = 0.1 #----------------> Multiplicador para aumentar las decimales

f1 = 1 #--------------------|\
w_a = 2*pi*f1 #-------------|-> Condiciones iniciales del sistema Xa
theta_a = 0 #---------------|/

f2 = 1.1 #------------------|\
w_b = 2*pi*f2 #-------------|-> Condiciones iniciales del sistema Xb
theta_b = 0 #---------------|/
###################################################################################
# Declaracion de variables de almacenamiento
px0 = [] #------> Variable de almacenamiento de la posicion del robot en x
py0 = [] #------> Variable de almacenamiento de la posicion del robot en y
theta0 = [] #---> Variable de almacenamiento de la orientacion del robot en θ

tiempo = [] #-----> Variable de almacenamiento del tiempo del robot

xi0 = [] #--------> Variable de almacenamiento del estado auxiliar del robot
omega = [] #------> Variable de almacenamiento de la velocidad angular del robot

xd1 = [] #--------> Variable de almacenamiento de datos de xd
yd1 = [] #--------> Variable de almacenamiento de datos de yd

cripto_xd0 = [] #--------> Variable de almacenamiento de datos de cripto_xd
cripto_yd0 = [] #--------> Variable de almacenamiento de datos de cripto_yd

cripto_vx0 = [] #--------> Variable de almacenamiento de datos de cripto_vx
cripto_vy0 = [] #--------> Variable de almacenamiento de datos de cripto_vy

#decripto_xd0 = [] #--------> Variable de almacenamiento de datos de decripto_xd
#decripto_yd0 = [] #--------> Variable de almacenamiento de datos de decripto_yd
###################################################################################
# Funcion para determinar la carga total de las baterias el robot
def bateria(data):
    global bat
    bat = data.voltage
###################################################################################
# Funcion para determinar la orientacion del robot y su matriz de rotacion
def orientacion(data):
    global R,a11,a12,a21,a22,quatx1,quaty1,quatz1,unitq1

    quatx1 = data.orientation.x
    quaty1 = data.orientation.y
    quatz1 = data.orientation.z
    unitq1 = data.orientation.w

    # Calculo de la matriz de rotacion
    a11 = 2*(unitq1**2 + quatx1**2) - 1
    a12 = 2*(quatx1*quaty1 - unitq1*quatz1)

    a21 = 2*(quatx1*quaty1 + unitq1*quatz1)
    a22 = 2*(unitq1**2 + quaty1**2) - 1     
    
    R = array([[a11,a12], [a21,a22]]) #-----------> Matriz de rotacion R
###################################################################################
# Funcion para determinar la posicion px y py del robot
def posicion(data):
    global px,py

    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
###################################################################################
# Funcion para la ejecucion del programa y la ley de control del robot
def listener():
    global xi,theta_a,theta_b,w_a,w_b

    rospy.init_node('listener',anonymous=True) #---> Inicializacion del nodo
    rospy.Subscriber('/battery',BatteryState,bateria) #---> Suscripcion al topico /battery
    rospy.Subscriber('/odom',Odometry,posicion) #---------> Suscripcion al topico /odom
    rospy.Subscriber('/imu',Imu,orientacion) #------------> Suscripcion al topico /imu
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #---> Publicacion al nodo /cmd_vel

    ###################################################################################
    # Declaracion de puertos de comunicacion y creacion del TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Definicion del host 
    host = '192.168.0.108' #-----> IP del propio robot

    # Declaracion del puerto de communicacion
    port = 8080

    message_struct = struct.Struct('!ffff') #-----> Tamaño del vector de mensaje

    # Conexion del socket al puerto donde el server esta esperando 
    server_address = ((host, port))

    print("Conectando...")

    sock.connect(server_address)

    hz = 50 #-----------------> Hertz (hz) de frecuencia
    tau = 1/hz #--------------> Tamaño de paso del tiempo

    T = tau*dT #--------------> Decimales aumentadas para tau

    rate = rospy.Rate(hz) #---> Obtencion de la frecuencia del robot
    to = rospy.get_time() #---> Obtencion del tiempo de ejecucion del programa en el robot

    c11 = 3 #-----------------> Ganancia proporcional c11 de px
    c21 = 3 #-----------------> Ganancia proporcional c21 de py

    c12 = 2 #-----------------> Ganancia derivativa c12 de vx
    c22 = 2 #-----------------> Ganancia derivativa c22 de vy

    while not rospy.is_shutdown() and rospy.get_time() - to <= 50:

        vel = Twist()

        if bat <= 9 and bat != 0:

            print('Bateria baja:', bat/3)
            break

        else:
###################################################################################        
        # Dinamica del sistema hipercaotico
            theta_a = A*sin(w_a)  
            w_a = (w_a*b) + c*(eta*theta_a + theta_b)

            theta_b = A*sin(w_b) 
            w_b = (w_b*b) + c*(eta*theta_b + theta_a)
###################################################################################        
        # Trayectoria del robot
            xi0.append(xi) #-----> Adquisición de datos para la variable de almacenamiento xi0

            theta = math.atan2(R[1,0],R[0,0])
            t = rospy.get_time() - to

            xd = 0.8*sin(2*(pi/50)*t)
            yd = 0.8*sin(2*(pi/25)*t)
        
            xdp = 0.8*2*(pi/50)*cos(2*(pi/50)*t) 
            ydp = 0.8*2*(pi/25)*cos(2*(pi/25)*t)

            xdpp = -0.8*((2*(pi/50))**2)*sin(2*(pi/50)*t)
            ydpp = -0.8*((2*(pi/25))**2)*sin(2*(pi/25)*t)
###################################################################################       
        # Encriptación de señales de la trayectoria
            cripto_xd = theta_a + px
            cripto_xd = cripto_xd*T
    
            cripto_yd = theta_b + py
            cripto_yd = cripto_yd*T
###################################################################################       
        # Desencriptación de señales de trayectoria
            #decripto_xd = cripto_xd/T - theta_a
            #decripto_yd = cripto_yd/T - theta_b
    
            #decripto_xdp = cripto_xd/T - theta_ap
            #decripto_ydp = cripto_yd/T - theta_bp
###################################################################################       
        # Adquisición de datos en las variables de almacenamiento
            xd1.append(xd)
            yd1.append(yd)

            px0.append(px)
            py0.append(py)
            theta0.append(theta)

            tiempo.append(round(t,3))

            cripto_xd0.append(cripto_xd)
            cripto_yd0.append(cripto_yd)

            #decripto_xd0.append(decripto_xd)
            #decripto_yd0.append(decripto_yd)
###################################################################################       
        # Control por retroalimentacion dinamica
            z1p = xi*cos(theta)
            z2p = xi*sin(theta)
            u1 = -c11*(-xd + px) - c12*(-xdp + z1p) + xdpp
            u2 = -c21*(-yd + py) - c22*(-ydp + z2p) + ydpp
            w = (u2*cos(theta) - u1*sin(theta))/xi
###################################################################################       
        # Encriptación de velocidades vx y vy del robot y envío de datos 
            cripto_vx = theta_a + z1p
            cripto_vx = cripto_vx*T

            cripto_vy = theta_b + z2p
            cripto_vy = cripto_vy*T

            cripto_vx0.append(cripto_vx)
            cripto_vy0.append(cripto_vy)

            param = (cripto_xd,cripto_yd,cripto_vx,cripto_vy) #-------> Variable para envío de datos

            omega.append(w)
            sock.sendall(message_struct.pack(*param))
###################################################################################
            #vel = Twist()
            vel.linear.x = xi #---> Entrada de control xi
            vel.angular.z = w #---> Entrada de control w
            pub.publish(vel)

            xi = xi + tau*( u1*cos(theta) + u2*sin(theta) ) #----> Solución discreta de xi

            print('Tiempo:', rospy.get_time() - to,'Seg','Bateria:', bat,'V')
            rate.sleep()
###################################################################################
# Paro del robot   
    vel.linear.x = 0 #---------> Velocidad lineal en 0
    vel.angular.z = 0 #--------> Velocidad angular en 0
    pub.publish(vel) #---------> Ejecución de las nuevas velocidades
    sock.close() #-------------> Cierre de conexión con el server
###################################################################################
# Graficacion de estados y plano de fase del robot 
    fig = plt.figure()
    plt.title('Posición del sistema')
    plt.plot(xd1,yd1,'m',px0,py0,'k',lw=1.5)
    plt.xlabel('$x(t)$')
    plt.ylabel('$y(t)$')
    plt.grid(True)
    plt.show()
    plt.savefig('plano_fase_sincro_lineal_master.eps')

    #fig = plt.figure()
    #plt.title('Error')
    #plt.plot(tiempo,px0-xd1,'m',tiempo,py0-yd1,'b')
    #plt.xlabel('$t$')
    #plt.ylabel('$e_x(t)$')
    #plt.grid(True)
    #plt.show()
    #plt.savefig('error_sincro_lineal_master.eps')
###################################################################################
# Exportacion de datos para post-procesamiento
    datos = '/home/husarion/husarion_ws/src/controlador/scripts/datos_sincro_lineal_maestro_dinamico.mat'
    sio.savemat(datos,{'t':tiempo,'px_m':px0,'py_m':py0,'theta_m':theta0,'v_m':xi0,'w_m':omega,'cripto_xd':cripto_xd0,'cripto_yd':cripto_yd0,'cripto_vx':cripto_vx0,'cripto_vy':cripto_vy0})
    #rospy.spin()
    
if __name__=='__main__':
    listener()
