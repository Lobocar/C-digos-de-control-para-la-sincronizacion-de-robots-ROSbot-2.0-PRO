#!/usr/bin/env python
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

xi = array([0.1,0.1]) #---------> Condicion inicial del estado auxiliar h
S = array([[0,-1],[1,0]]) #------> Matriz de operacion auxiliar S

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
# Declaracion de variables del acoplamiento dinamico
h1 = array([0,0]) #----> Condicion inicial del estado h1 del acoplamiento dinamico
h2 = array([0,0]) #----> Condicion inicial del estado h2 del acoplamiento dinamico
#h1 = h[0]
#h2 = h[1]
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

f1p = 1.0000000000001 #-----|\
w_ap = 2*pi*f1p #-----------|-> Condiciones iniciales del sistema Xap
theta_ap = 0 #--------------|/

f2p = 1.1 #-----------------|\
w_bp = 2*pi*f2p #-----------|-> Condiciones iniciales del sistema Xbp
theta_bp = 0 #--------------|/
###################################################################################
# Declaracion de variables de almacenamiento
px0 = [] #--------> Variable de almacenamiento de la posicion del robot en x
py0 = [] #--------> Variable de almacenamiento de la posicion del robot en y
theta0 = [] #-----> Variable de almacenamiento de la orientacion del robot en θ

tiempo = [] #-----> Variable de almacenamiento del tiempo del robot

xix = [] #---------> Variable de almacenamiento del estado auxiliar xix del robot
xiy = [] #---------> Variable de almacenamiento del estado auxiliar xiy del robot

hx = [] #---------> Variable de almacenamiento del estado auxiliar hx del acoplamiento dinamico
hy = [] #---------> Variable de almacenamiento del estado auxiliar hy del acoplamiento dinamico

velo = [] #-------> Variable de almacenamiento de la velocidad lineal del robot
omega = [] #------> Variable de almacenamiento de la velocidad angular del robot

#ex = [] #---------> Variable de almacenamiento del error del robot en x
#ey = [] #---------> Variable de almacenamiento del error del robot en y

xd1 = [] #--------> Variable de almacenamiento de datos de xd
yd1 = [] #--------> Variable de almacenamiento de datos de yd

#cripto_xd0 = [] #--------> Variable de almacenamiento de datos de cripto_xd
#cripto_yd0 = [] #--------> Variable de almacenamiento de datos de cripto_yd

decripto_xd0 = [] #--------> Variable de almacenamiento de datos de decripto_xd
decripto_yd0 = [] #--------> Variable de almacenamiento de datos de decripto_yd

#decripto_vx0 = [] #--------> Variable de almacenamiento de datos de decripto_vx
#decripto_vy0 = [] #--------> Variable de almacenamiento de datos de decripto_vy
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
    global xi,h1,h2,theta_a,theta_b,w_a,w_b,theta_ap,theta_bp,w_ap,w_bp

    rospy.init_node('listener',anonymous=True) #---> Inicializacion del nodo
    rospy.Subscriber('/battery',BatteryState,bateria) #---> Suscripcion al topico /battery
    rospy.Subscriber('/odom',Odometry,posicion) #---------> Suscripcion al topico /odom
    rospy.Subscriber('/imu',Imu,orientacion) #------------> Suscripcion al topico /imu
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #---> Publicacion al nodo /cmd_vel

###################################################################################
    # Declaracion de puertos de comunicacion y creacion del TCP/IP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Definición del host y del puerto de comunicación
    host = '192.168.0.108'  # IP del servidor
    port = 8080

    # Bind the socket to the port
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    message_struct = struct.Struct('!ff')

    # Listen for incoming connections
    server_socket.listen(1)

    # Wait for a connection
    print('Esperando conexión')
    connection, client_address = server_socket.accept()
    print('Conexión desde', client_address)

    hz = 50 #-----------------> Hertz (hz) de frecuencia
    tau = 1/hz #--------------> Tamaño de paso del tiempo

    T = tau*dT #--------------> Decimales aumentadas para tau

    rate = rospy.Rate(hz) #---> Obtencion de la frecuencia del robot
    to = rospy.get_time() #---> Obtencion del tiempo de ejecucion del programa en el robot

    lamb1 = 2 #---------------------------> Ganancia de orientacion
    lamb2 = diag([1,1]) #-------------------> Ganancia de velocidad
    lamb3 = diag([1,1]) #---------------> Ganancia de posicion

    k = 2 #---------------> Constante de acoplamiento
    b1 = -1 #--------------> Variable de fase o antifase

    G1 = diag([1,1]) #--------------> Matriz paramétrica de estabilización G1  
    G2 = diag([1,1]) #--------------> Matriz paramétrica de estabilización G2  

    #a1 = 50 #-----------------> Frecuencia de la trayectoria en xd
    #a2 = a1/2 #---------------> Frecuencia de la trayectoria en yd

    while not rospy.is_shutdown() and rospy.get_time() - to <= 50:

        vel = Twist()

        if bat <= 9 and bat != 0:

            print('Bateria baja:', bat/3)
            break

        else:
            data = connection.recv(message_struct.size)
            if not data:
                break

            # Process the received data and send the response
            current_pose = message_struct.unpack(data)
###################################################################################        
        # Dinamica del sistema hipercaotico
            theta_a = A*sin(w_a)  
            w_a = (w_a*b) + c*(eta*theta_a + theta_b)

            theta_b = A*sin(w_b) 
            w_b = (w_b*b) + c*(eta*theta_b + theta_a)
###################################################################################        
        # Dinamica derivada del sistema hipercaotico
            theta_ap = A*sin(w_ap)
            w_ap = (w_ap*b) + c*(eta*theta_ap + theta_bp)
    
            theta_bp = A*sin(w_bp)
            w_bp = (w_bp*b) + c*(eta*theta_bp + theta_ap)
###################################################################################        
        # Trayectoria del robot
            theta = math.atan2(R[1,0],R[0,0])
            r = array(R[:,0])
            t = rospy.get_time() - to

            cripto_xd = current_pose[0] #------> Posición en x del robot maestro encriptado
            cripto_yd = current_pose[1] #------> Posición en y del robot maestro encriptado
            #cripto_vx = current_pose[2] #------> Velocidad en x del robot maestro encriptado
            #cripto_vy = current_pose[3] #------> Velocidad en y del robot maestro encriptado
        
            #xdp = current_pose[2]
            #ydp = current_pose[3]

            xdpp = 0 #-0.8*((2*(pi/a1))**2)*sin(2*(pi/a1)*t)
            ydpp = 0 #-0.8*((2*(pi/a2))**2)*sin(2*(pi/a2)*t)

            p = array([px,py]) #----------> Vector de posicion px y py del robot
            #pd = array([xd,yd]) #---------> Vector de posicion deseada
            #vd = array([xdp,ydp]) #-------> Vector de velocidad deseada
            #ad = array([xdpp,ydpp]) #-----> Vector de aceleracion deseada
###################################################################################       
        # Desencriptación de señales de trayectoria
            xd = cripto_xd/T - theta_a #------> Posición en x del robot maestro desencriptado
            yd = cripto_yd/T - theta_b #------> Posición en y del robot maestro desencriptado

            #xdp = cripto_vx/T - theta_a #------> Velocidad en x del robot maestro desencriptado
            #ydp = cripto_vy/T - theta_b #------> Velocidad en y del robot maestro desencriptado

            pd = array([xd,yd]) #---------> Vector de posicion deseada
            #vd = array([xdp,ydp]) #-------> Vector de velocidad deseada
    
            decripto_xd = cripto_xd/T - theta_ap #------> Posición en x del robot maestro desencriptado con otra condición inicial
            decripto_yd = cripto_yd/T - theta_bp #------> Posición en y del robot maestro desencriptado con otra condición inicial
            #decripto_vx = cripto_vx/T - theta_ap #------> Velocidad en x del robot maestro desencriptado con otra condición inicial
            #decripto_vy = cripto_vy/T - theta_bp #------> Velocidad en y del robot maestro desencriptado con otra condición inicial
###################################################################################       
        # Adquisición de datos en las variables de almacenamiento
            xd1.append(xd)
            yd1.append(yd)

            px0.append(px)
            py0.append(py)
            theta0.append(theta)

            xix.append(xi[0])
            xiy.append(xi[1])

            hx.append(h1)
            hy.append(h2)

            #ex.append(round((px-xd),3))
            #ey.append(round((py-yd),3))

            tiempo.append(round(t,3))

            #cripto_xd0.append(cripto_xd)
            #cripto_yd0.append(cripto_yd)

            decripto_xd0.append(decripto_xd)
            decripto_yd0.append(decripto_yd)
            #decripto_vx0.append(decripto_vx)
            #decripto_vy0.append(decripto_vy)
###################################################################################       
        # Control no lineal
            h2p = -matmul(G1,h2) - matmul(G2,h1) + k*( pd + b1*p )

            xip = -matmul(lamb2,xi) + matmul(lamb3,h1) + h2p + matmul(lamb2,h2)
            
            W = -(matmul(transpose(xi),matmul(S,xip)))/(matmul(transpose(xi),xi))

            v = linalg.norm(xi)
            w = W - (lamb1*matmul(transpose(r),matmul(S,xi)))/linalg.norm(xi)

            velo.append(v)
            omega.append(w)
###################################################################################
            #vel = Twist()
            vel.linear.x = v #----> Entrada de control v
            vel.angular.z = w #---> Entrada de control w
            pub.publish(vel) #----> Ejecución de las velocidades

            xi = xi + tau*xip #------> Solucion para el estado auxiliar xi

            h1 = h1 + tau*h2 #-------> Solucion para el estado auxiliar h1
            h2 = h2 + tau*h2p #------> Solucion para el estado auxiliar h2

            #h = array([h1,h2])

            print('Tiempo:', rospy.get_time() - to,'Seg','Bateria:', bat,'V')
            rate.sleep()
###################################################################################
# Paro del robot   
    vel.linear.x = 0 #---------> Velocidad lineal en 0
    vel.angular.z = 0 #--------> Velocidad angular en 0
    pub.publish(vel) #---------> Ejecución de las nuevas velocidades
    connection.close() #-------> Cierre de conexión del socket
###################################################################################
# Graficacion de estados y plano de fase del robot 
    fig = plt.figure()
    plt.title('Posición del sistema')
    plt.plot(xd1,yd1,'m',px0,py0,'k',lw=1.5)
    plt.xlabel('$x(t)$')
    plt.ylabel('$y(t)$')
    plt.grid(True)
    plt.show()
    plt.savefig('plano_fase_sincro_no_lineal_slave.eps')

    #fig = plt.figure()
    #plt.title('Error')
    #plt.plot(tiempo,px0-xd1,'m',tiempo,py0-yd1,'b')
    #plt.xlabel('$t$')
    #plt.ylabel('$e_x(t)$')
    #plt.grid(True)
    #plt.show()
    #plt.savefig('error_sincro_no_lineal_slave.eps')
###################################################################################
# Exportacion de datos para post-procesamiento
    datos = '/home/husarion/husarion_ws/src/controlador/scripts/datos_sincro_no_lineal_esclavo_dinamico.mat'
    sio.savemat(datos,{'t':tiempo,'px_e':px0,'py_e':py0,'theta_e':theta0,'xix_e':xix,'xiy_e':xiy,'hx_e':hx,'hy_e':hy,'v_e':velo,'w_e':omega,'decripto_xd':decripto_xd0,'decripto_yd':decripto_yd0}) #,'decripto_vx':decripto_vx0,'decripto_vy':decripto_vy0})
    #rospy.spin()
    
if __name__=='__main__':
    listener()
