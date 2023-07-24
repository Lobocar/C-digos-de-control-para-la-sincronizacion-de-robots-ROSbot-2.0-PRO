%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Programa realizado por Carlos Andres Villalobos Aranda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cierre de ventanas emergentes y limpieza de variables
close all;
clc;
clear all;
%###############################################################################
% Condiciones iniciales, parametros del robot y variables auxiliares
tiempo = 50; %---------------> Tiempo de simulación
tau = 0.01; %----------------> Tamaño de paso
N = tiempo/tau; %------------> Numero de pasos
dT = 0.1; %------------------> Multiplicador para aumentar las decimales
T = tau*dT; %----------------> Decimales aumentadas para tau

xd(1) = 0; %---------------|-> Condiciones iniciales de la trayectoria
yd(1) = 0; %--------------/

c11 = 3; %-------------------> Ganancia proporcional c11 de px
c21 = 3; %-------------------> Ganancia proporcional c21 de py
c12 = 3; %-------------------> Ganancia derivativa c12 de vx
c22 = 3; %-------------------> Ganancia derivativa c22 de vy

%c11_a = 3; %-----------------> Constante proporcional c11 de sincronización de px
%c21_a = 3; %-----------------> Constante proporcional c21 de sincronización de py
%c12_a = 3; %-----------------> Constante derivativa c12 de sincronización de vx
%c22_a = 3; %-----------------> Constante derivativa c22 de sincronización de vy

r = 0.0425; %----------------> Radio de las ruedas
L = 0.192; %---------------> Distancia entre ruedas

x_m(1) = 1; %----------------> Condición inicial en x del robot maestro
y_m(1) = 1; %----------------> Condición inicial en y del robot maestro
theta_m(1) = 0; %------------> Condición inicial en theta del robot maestro

xi_m(1) = 0.1; %-------------> Condición inicial xi del compensador dinámico del robot maestro

x_e(1) = -1; %---------------> Condición inicial en x del robot esclavo
y_e(1) = -1; %---------------> Condición inicial en y del robot esclavo
theta_e(1) = 0; %------------> Condición inicial en theta del robot esclavo

xi_e(1) = 0.1; %-------------> Condición inicial xi del compensador dinámico del robot esclavo

hx = [0;0]; %----------------> Condición inicial hx del acoplamiento dinamico
hxp = [0;0]; %---------------> Condición inicial hxp del acoplamiento dinamico

hy = [0;0]; %----------------> Condición inicial hy del acoplamiento dinamico
hyp = [0;0]; %---------------> Condición inicial hyp del acoplamiento dinamico

h = [hx;hxp;hy;hyp]; %---------------> Condición inicial h del acoplamiento dinamico

alpha = 1;
gamma1 = 0.1;
gamma2 = 5;

k = 15; %---------------------> Fuerza de acoplamiento entre robots

b1 = -1; %---------------------> Constante de fase o antifase beta 1
b2 = -1; %---------------------> Constante de fase o antifase beta 2

G = [-alpha,1;-gamma1,-gamma2];

B1 = [0,b1,0,b1,0,0,0,0;... %------> Matriz de acoplamiento B1
      0,0,0,0,0,b2,0,b2]; 
      
B2 = [0,0,0,0;...           %------> Matriz de acoplamiento B2
     1,0,0,0;...
      0,0,0,0;...
      0,1,0,0;...
      0,0,0,0;...
      0,0,1,0;...
      0,0,0,0;...
      0,0,0,1];

B2_hx = [B2(1,:);B2(2,:)]; %-------> Matriz de acoplamiento B2 para hx
B2_hxp = [B2(3,:);B2(4,:)]; %------> Matriz de acoplamiento B2 para hxp
B2_hy = [B2(5,:);B2(6,:)]; %-------> Matriz de acoplamiento B2 para hy
B2_hyp = [B2(7,:);B2(8,:)]; %------> Matriz de acoplamiento B2 para hyp
      
t(1) = 0; %----------------> Condición inicial del tiempo continuo
i(1) = 0; %----------------> Variable auxiliar de conteo
%###############################################################################
% Condiciones iniciales y parámetros del sistema hipercaótico
b = -0.5; %----------------> 1er parámetro
c = 4; %-------------------> 2do parámetro
eta = -0.5; %--------------> 3er parámetro

A = 2000; %---------------->Amplitud para encriptación de señales

f1(1) = 1; %--------------------|\
w_a(1) = 2*pi*f1(1); %----------|->Condiciones iniciales del sistema Xa
theta_a(1) = 0; %---------------|/

f2(1) = 1.1; %------------------|\
w_b(1) = 2*pi*f2(1); %----------|->Condiciones iniciales del sistema Xb
theta_b(1) = 0; %---------------|/

f1p(1) = 1.0000000000001; %-----|\
w_ap(1) = 2*pi*f1p(1); %--------|->Condiciones iniciales del sistema Xap
theta_ap(1) = 0; %--------------|/

f2p(1) = 1.1; %-----------------|\
w_bp(1) = 2*pi*f2p(1); %--------|->Condiciones iniciales del sistema Xbp
theta_bp(1) = 0; %--------------|/

cripto_xd(1) = 0;
cripto_yd(1) = 0;
cripto_vx(1) = 0;
cripto_vy(1) = 0;
%###############################################################################
% Calculo de trayectorias y dinámicas del robot y del sistema hipercaótico
for n = 1:N
    
    xd(n+1) = 0.8*sin(2*pi/50*t(n)); %------------|->Trayectoria del robot
    yd(n+1) = 0.8*sin(2*pi/25*t(n)); %-----------/
    
    xdp = 0.8*2*pi/50*cos(2*pi/50*t(n)); %--------|->Trayectoria derivada
    ydp = 0.8*2*pi/25*cos(2*pi/25*t(n)); %-------/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sistema hipercaotico
    theta_a(n+1) = A*sin(w_a(n));  
    w_a(n+1) = (w_a(n)*b) + c*(eta*theta_a(n) + theta_b(n));  
    
    theta_b(n+1) = A*sin(w_b(n)); 
    w_b(n+1) = (w_b(n)*b) + c*(eta*theta_b(n) + theta_a(n)); 
%-------------------------------------------------------------------------------
% Sistema hipercaótico derivado
    theta_ap(n+1) = A*sin(w_ap(n));
    w_ap(n+1) = (w_ap(n)*b) + c*(eta*theta_ap(n) + theta_bp(n));
    
    theta_bp(n+1) = A*sin(w_bp(n));
    w_bp(n+1) = (w_bp(n)*b) + c*(eta*theta_bp(n) + theta_ap(n));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Encriptación de señales de trayectoria
    cripto_xd(n) = theta_a(n) + x_m(n);
    cripto_xd(n) = cripto_xd(n)*T;
    
    cripto_yd(n) = theta_b(n) + y_m(n);
    cripto_yd(n) = cripto_yd(n)*T;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Desencriptación de señales de trayectoria   
    decripto_xd(n) = cripto_xd(n)/T - theta_a(n);
    decripto_yd(n) = cripto_yd(n)/T - theta_b(n);
    
    decripto_xdp(n) = cripto_xd(n)/T - theta_ap(n);
    decripto_ydp(n) = cripto_yd(n)/T - theta_bp(n);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control y dinamicas del robot maestro
    z1p_m = xi_m(n)*cos(theta_m(n));
    z2p_m = xi_m(n)*sin(theta_m(n));
    
    cripto_vx(n) = theta_a(n) + z1p_m;
    cripto_vx(n) = cripto_vx(n)*T;
    
    cripto_vy(n) = theta_b(n) + z2p_m;
    cripto_vy(n) = cripto_vy(n)*T;
    
    u1_m = c11*(xd(n) - x_m(n)) + c12*(xdp - z1p_m);
    u2_m = c21*(yd(n) - y_m(n)) + c22*(ydp - z2p_m);
    
    w_m = (u2_m*cos(theta_m(n)) - u1_m*sin(theta_m(n)))/xi_m(n);
    wL_m = xi_m(n)/r - L*w_m/(2*r);
    wR_m = xi_m(n)/r + L*w_m/(2*r);
    vL_m(n) = r*wL_m;
    vR_m(n) = r*wR_m;
    
    x_m(n+1) = x_m(n) + tau*(z1p_m);
    y_m(n+1) = y_m(n) + tau*(z2p_m);
    theta_m(n+1) = theta_m(n) + tau*(w_m);
    xi_m(n+1) = xi_m(n) + tau*( u1_m*cos(theta_m(n)) + u2_m*sin(theta_m(n)) );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control y dinamicas del robot esclavo
    z1p_e = xi_e(n)*cos(theta_e(n));
    z2p_e = xi_e(n)*sin(theta_e(n));
    
    decripto_vx(n) = cripto_vx(n)/T - theta_a(n);
    decripto_vy(n) = cripto_vy(n)/T - theta_b(n);
    
    Ue = B1*h; %------> Entrada de control del robot esclavo (acoplamiento dinamico) 
    
    u1_e = Ue(1,:); %c11_a*(x_m(n) - x_e(n)) + c12_a*(z1p_m - z1p_e);
    u2_e = Ue(2,:); %c21_a*(y_m(n) - y_e(n)) + c22_a*(z2p_m - z2p_e);
    
    w_e = (u2_e*cos(theta_e(n)) - u1_e*sin(theta_e(n)))/xi_e(n);
    wL_e = xi_e(n)/r - L*w_e/(2*r);
    wR_e = xi_e(n)/r + L*w_e/(2*r);
    vL_e(n) = r*wL_e;
    vR_e(n) = r*wR_e;
    
    x_e(n+1) = x_e(n) + tau*(z1p_e);
    y_e(n+1) = y_e(n) + tau*(z2p_e);
    theta_e(n+1) = theta_e(n) + tau*(w_e);
    xi_e(n+1) = xi_e(n) + tau*( u1_e*cos(theta_e(n)) + u2_e*sin(theta_e(n)) );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dinamicas del sistema intermediario (auxiliar) para el acoplamiento dinamico
    e_x = decripto_xd(n) + b1*x_e(n); %----------------> Error en x del robot
    e_xp = decripto_vx(n) + b1*z1p_e; %-----------> Error en xp del robot

    e_y = decripto_yd(n) + b2*y_e(n); %----------------> Error en y del robot
    e_yp = decripto_vy(n) + b2*z2p_e; %-----------> Error en yp del robot

    E = [e_x;e_xp;e_y;e_yp]; %--------> Vector de Error

    hx = hx + tau*( G*hx - k*B2_hx*E );
    
    hxp = hxp + tau*( G*hxp - k*B2_hxp*E );
    
    hy = hy + tau*( G*hy - k*B2_hy*E );
    
    hyp = hyp + tau*( G*hyp - k*B2_hyp*E );
    
    h = [hx;hxp;hy;hyp];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definición del tiempo y aumento de variables de conteo
    i(n+1) = i(n) + 1;
    t(n+1) = tau*i(n); %------------->Tiempo continuo
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Almacenamiento de variables de graficacion
    u1m(n) = u1_m;
    u1s(n) = u1_e;
    u2m(n) = u2_m;
    u2s(n) = u2_e;
    
    wm(n) = w_m;
    ws(n) = w_e;
    
    hx1(n) = hx(1,:);
    hx2(n) = hx(2,:);
    hy1(n) = hy(1,:);
    hy2(n) = hy(2,:);
    
    hxp1(n) = hxp(1,:);
    hxp2(n) = hxp(2,:);
    hyp1(n) = hyp(1,:);
    hyp2(n) = hyp(2,:);
    

end % end para MATLAB
%endfor % endfor para OCTAVE 
%##########################################################################
% GraficaciÃ³n de las trayectorias del robot mÃ³vil y sus seÃ±ales encriptadas
fuente = 14;
linea = 2;
%--------------------------------------------------------------------------
figure()
subplot(3,1,1)
hold on
plot(t,x_m,'r','LineWidth',linea)
plot(t,x_e,'b','LineWidth',linea)
plot(t,xd,'k--','LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$x(t)$ [m]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$x_{m}(t)$','$x_{s}(t)$','$x_{d}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

subplot(3,1,2)
hold on
plot(t,y_m,'Color',[1 0.2235 0],'LineWidth',linea)
plot(t,y_e,'Color',[0.5058 0 1],'LineWidth',linea)
plot(t,yd,'k--','LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$y(t)$ [m]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$y_{m}(t)$','$y_{s}(t)$','$y_{d}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

subplot(3,1,3)
hold on
plot(t,theta_m,'m','LineWidth',linea)
plot(t,theta_e,'g','LineWidth',linea)
xlim([0 50])
ylim([-13 3])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\theta (t)$ [rad]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$\theta_{m}(t)$','$\theta_{s}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graficacion de velocidades
figure()
subplot(2,1,1)
hold on
plot(t,xi_m,'r','LineWidth',linea)
plot(t,xi_e,'b','LineWidth',linea)
xlim([0 50])
ylim([-1 2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\nu (t)$ [m/s]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$\nu_{m}(t)$','$\nu_{s}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

subplot(2,1,2)
hold on
plot(t(1:N),wm,'m','LineWidth',linea)
plot(t(1:N),ws,'g','LineWidth',linea)
xlim([0 50])
ylim([-8 8])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\omega (t)$ [rad/s]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$\omega_{m}(t)$','$\omega_{s}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)
%print -depsc Estados_ROSbot.pdf % Comando para exportar imagen en formato EPS
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graficacion del plano de fase 
figure()
hold on
plot(x_m,y_m,'r','LineWidth',linea)
plot(x_e,y_e,'b','LineWidth',linea)
plot(xd,yd,'k--','LineWidth',linea)
xlim([-1.5 1.5])
ylim([-1.5 1.5])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$x(t)$ [m]','Interpreter','latex','fontsize',fuente)
ylabel('$y(t)$ [m]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('{\boldmath $p$}$_{m}(t)$','{\boldmath $p$}$_{s}(t)$','{\boldmath $p$}$_{d}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graficacion del error del maestro con la trayectoria y el maestro con el esclavo
figure()
subplot(2,1,1)
hold on
plot(t,x_m-xd,'r','LineWidth',linea)
plot(t,y_m-yd,'b','LineWidth',linea)
xlim([0 50])
ylim([-0.5 1])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('{\boldmath $\tilde{p}$}$(t)$ [m]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$\tilde{x}(t)$','$\tilde{y}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

subplot(2,1,2)
hold on
plot(t,x_m-x_e,'m','LineWidth',linea)
plot(t,y_m-y_e,'g','LineWidth',linea)
xlim([0 50])
ylim([-0.5 1])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('{\boldmath $\bar{p}$}$(t)$ [m]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$\bar{x}(t)$','$\bar{y}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graficacion de los estados del acoplamiento dinamico 
figure()
subplot(2,1,1)
hold on
plot(t(1:N),hx1,'LineWidth',linea)
plot(t(1:N),hx2,'LineWidth',linea)
plot(t(1:N),hy1,'LineWidth',linea)
plot(t(1:N),hy2,'LineWidth',linea)
xlim([0 50])
ylim([-6 2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('{\boldmath$h$}$_{p}(t)$ [m]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$h_{x_{1}}(t)$','$h_{x_{2}}(t)$','$h_{y_{1}}(t)$','$h_{y_{2}}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

subplot(2,1,2)
hold on
plot(t(1:N),hxp1,'LineWidth',linea)
plot(t(1:N),hxp2,'LineWidth',linea)
plot(t(1:N),hyp1,'LineWidth',linea)
plot(t(1:N),hyp2,'LineWidth',linea)
xlim([0 50])
ylim([-2 6])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('{\boldmath$\dot{h}$}$_{p}(t)$ [m/s]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$\dot{h}_{x_{1}}(t)$','$\dot{h}_{x_{2}}(t)$','$\dot{h}_{y_{1}}(t)$','$\dot{h}_{y_{2}}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Graficaci?n del encriptado de se?al del robot
figure()
subplot(4,1,1)
plot(t,x_m,'LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$x_{m}(t)$ [m]','Interpreter','latex','fontsize',fuente)
%LEG1 = legend('$\bar{x}(t)$','$\bar{y}(t)$')
%set(LEG1,'Interpreter','latex');
%set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
%set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

% subplot(4,2,2)
% plot(t,y_m,'LineWidth',linea)
% xlabel('t[s]','fontsize',fuente)
% ylabel('y_m(t)[m]','fontsize',fuente)
% title('Mensaje original y_m(t)')
% grid on
% set(gca,'fontsize',fuente)

subplot(4,1,2)
plot(t(1:N),cripto_xd,'LineWidth',linea)
xlim([0 50])
ylim([-2.5 2.5])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\check{x}_{m}(t)$ [m]','Interpreter','latex','fontsize',fuente)
% LEG1 = legend('$\bar{x}(t)$','$\bar{y}(t)$')
% set(LEG1,'Interpreter','latex');
% set(LEG1,'FontSize',fuente);
% %set(LEG1,'Location','northeast')
% set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

% subplot(4,2,4)
% plot(t(1:N),cripto_yd,'LineWidth',linea)
% xlabel('t[s]','fontsize',fuente)
% ylabel('y_m(t) + caos','fontsize',fuente)
% title('Mensaje encriptado y_m(t) + caos')
% grid on
% set(gca,'fontsize',fuente)

subplot(4,1,3)
plot(t(1:N),decripto_xd,'LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\hat{x}_{m}(t)$ [m]','Interpreter','latex','fontsize',fuente)
% LEG1 = legend('$\bar{x}(t)$','$\bar{y}(t)$')
% set(LEG1,'Interpreter','latex');
% set(LEG1,'FontSize',fuente);
% %set(LEG1,'Location','northeast')
% set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

% subplot(4,2,6)
% plot(t(1:N),decripto_yd,'LineWidth',linea)
% xlabel('t[s]','fontsize',fuente)
% ylabel('y_m(t) - caos','fontsize',fuente)
% title('Mensaje recuperado y_m(t) - caos')
% grid on
% set(gca,'fontsize',fuente)

subplot(4,1,4)
plot(t(1:N),decripto_xdp,'LineWidth',linea)
xlim([0 50])
ylim([-5000 5000])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\breve{x}_{m}(t)$ [m]','Interpreter','latex','fontsize',fuente)
% LEG1 = legend('$\bar{x}(t)$','$\bar{y}(t)$')
% set(LEG1,'Interpreter','latex');
% set(LEG1,'FontSize',fuente);
% %set(LEG1,'Location','northeast')
% set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

% subplot(4,2,8)
% plot(t(1:N),decripto_ydp,'LineWidth',linea)
% xlabel('t[s]','fontsize',fuente)
% ylabel('y_m(t) - caos_d','fontsize',fuente)
% title('Mensaje recuperado con diferente condiciˆun inicial')
% grid on
% set(gca,'fontsize',fuente)
% print -depsc2 -color sim_encriptado.eps % Comando para exportar imagen en formato EPS
