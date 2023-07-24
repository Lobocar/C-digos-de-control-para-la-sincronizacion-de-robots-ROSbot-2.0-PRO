% Programa realizado por Carlos Andr茅s Villalobos Aranda
%###############################################################################
close all;
clc;
clear all;
%###############################################################################
% Declaraci贸n de variables regulares
r = 0.0425; %----------------> Radio de las ruedas
L = 0.192; %---------------> Distancia entre ruedas
vR = 1; %-----------------Velocidad lineal de las ruedas derechas del robot
vL = 1; %-------------Velocidad lineal de las ruedas izquierdas del robot
%###############################################################################
% Declaraci贸n de variables del robot maestro
vm = (vR + vL)/2; %-------------------------------Velocidad lineal del robot maestro
wm = (vR - vL)/L; %------------------------------Velocidad angular del robot maestro

x0m = 1; %------------------------Estado x(t) "Posici贸n x" del robot maestro inicial
y0m = 1; %------------------------Estado y(t) "Posici贸n y" del robot maestro inicial
theta0m = 1; %------------Estado 胃(t) "Posici贸n angular 胃" del robot maestro inicial
%###############################################################################
% Declaraci贸n de variables del robot esclavo
vs = (vR + vL)/2; %-------------------------------Velocidad lineal del robot esclavo
ws = (vR - vL)/L; %------------------------------Velocidad angular del robot esclavo

x0s = -1; %------------------------Estado x(t) "Posici贸n x" del robot esclavo inicial
y0s = -1; %------------------------Estado y(t) "Posici贸n y" del robot esclavo inicial
theta0s = 1; %------------Estado 胃(t) "Posici贸n angular 胃" del robot esclavo inicial
%###############################################################################
% Declaraci贸n de variables auxiliares
tau = 0.01; %-------------------------------------Tama帽o de paso del robot
j = 100/tau; %-------------------------N煤mero de iteraciones de la simulaci贸n
i = 0; %--------------------------------------------------Contador auxiliar
dT = 0.1; %------------------> Multiplicador para aumentar las decimales
T = tau*dT; %----------------> Decimales aumentadas para tau
%###############################################################################
% Condiciones iniciales y par醡etros del sistema hiperca髏ico
b = -0.5; %----------------> 1er par醡etro
sus = 4; %-------------------> 2do par醡etro
eta = -0.5; %--------------> 3er par醡etro

A = 2000; %---------------->Amplitud para encriptaci髇 de se馻les

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
%###############################################################################
%###############################################################################
% Declaraci贸n de vectores y matrices del robot maestro
Vm = [0 -wm vm;wm 0 0;0 0 0]; %-------------------------------Matriz de entrada
pm = [x0m;y0m]; %-------------------------------------------Vector de posici贸n
Rm = [cos(theta0m) -sin(theta0m);sin(theta0m) cos(theta0m)]; %Matriz de rotaci贸n
Hm = [Rm pm;0 0 1]; %-----------------------------------------Matriz homog茅nea
Hsm = zeros(3,3,j); %-------------------------Resultado num茅rico del sistema
xim = [0.1;0.1]; %---------------------------------------------Vector auxiliar
lambdam = 2; %----------------------------------------Ganancia de orientaci锟絬n
Lambda2m = diag([1,1]); %-------------------------------Ganancia de velocidad
Lambda3m = diag([1,1]); %--------------------------------Ganancia de posici锟絬n
%###############################################################################
% Declaraci贸n de vectores y matrices del robot esclavo
Vs = [0 -ws vs;ws 0 0;0 0 0]; %-------------------------------Matriz de entrada
ps = [x0s;y0s]; %-------------------------------------------Vector de posici贸n
Rs = [cos(theta0s) -sin(theta0s);sin(theta0s) cos(theta0s)]; %Matriz de rotaci贸n
Hs = [Rs ps;0 0 1]; %-----------------------------------------Matriz homog茅nea
Hss = zeros(3,3,j); %-------------------------Resultado num茅rico del sistema
xis = [0.1;0.1]; %---------------------------------------------Vector auxiliar
lambdas = 2; %----------------------------------------Ganancia de orientaci锟絬n
Lambda2s = diag([1,1]); %-------------------------------Ganancia de velocidad
Lambda3s = diag([1,1]); %--------------------------------Ganancia de posici锟絬n
%###############################################################################
% Declaraci贸n de vectores y matrices del acoplamiento din坔mico
h1 = [0;0];
h2 = [0;0];
G1 = diag([1,1]); %----------> Par坔metro del acoplamiento din坔mico G1
G2 = diag([1,1]); %----------> Par坔metro del acoplamiento din坔mico G2
c = 0.5; %----------------------> Ganancia del acoplamiento din坔mico
b1 = -1; %--------------------> Constante unitaria de fase
%###############################################################################
% Declaraci贸n de vectores y matrices auxiliares
e1 = [1;0];
e2 = [0;1];
S = [e2 -e1]; %----------------------Matriz auxiliar de vectores unitarios
%###############################################################################
% C谩lculo num茅rico del comportamiento del sistema discretizado
for k = 1:1:j
    
    t(k) = tau*i; %------------------------------------Tiempo de simulaci贸n
    
    pxm(k) = Hm(1,3); %-------------------------------Posici坲n en x del robot maestro
    pym(k) = Hm(2,3); %-------------------------------Posici贸n en y del robot maestro
    
    pxs(k) = Hs(1,3); %-------------------------------Posici贸n en x del robot esclavo
    pys(k) = Hs(2,3); %-------------------------------Posici贸n en y del robot esclavo
    
    xd(k) = 0.8*sin(2*pi/50*t(k)); %------------|->Trayectoria del robot
    yd(k) = 0.8*sin(2*pi/25*t(k)); %-----------/
    
    xdp = 0.8*2*pi/50*cos(2*pi/50*t(k)); %--------|->Trayectoria derivada
    ydp = 0.8*2*pi/25*cos(2*pi/25*t(k)); %-------/
    
    xdpp = -0.8*((2*pi/50)^(2))*sin(2*pi/50*t(k)); %----|->Trayectoria derivada 2
    ydpp = -0.8*((2*pi/25)^(2))*sin(2*pi/25*t(k)); %----/
    
    %pm = [pxm(k);pym(k)]; %-------------------------> Vector de posici贸n del robot maestro
    ps = [pxs(k);pys(k)]; %-------------------------> Vector de posici贸n del robot esclavo
    
    pd = [xd(k);yd(k)]; %---------------------------> Posici锟絬n deseada
    vd = [xdp;ydp]; %-------------------------------> Velocidad deseada
    ad = [xdpp;ydpp]; %-----------------------------> Aceleracion deseada (vdp)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sistema hipercaotico
    theta_a(k+1) = A*sin(w_a(k));  
    w_a(k+1) = (w_a(k)*b) + sus*(eta*theta_a(k) + theta_b(k));  
    
    theta_b(k+1) = A*sin(w_b(k)); 
    w_b(k+1) = (w_b(k)*b) + sus*(eta*theta_b(k) + theta_a(k)); 
%-------------------------------------------------------------------------------
% Sistema hiperca髏ico derivado
    theta_ap(k+1) = A*sin(w_ap(k));
    w_ap(k+1) = (w_ap(k)*b) + sus*(eta*theta_ap(k) + theta_bp(k));
    
    theta_bp(k+1) = A*sin(w_bp(k));
    w_bp(k+1) = (w_bp(k)*b) + sus*(eta*theta_bp(k) + theta_ap(k));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Encriptaci髇 de se馻les de trayectoria
    cripto_xd(k) = theta_a(k) + pxm(k);
    cripto_xd(k) = cripto_xd(k)*T;
    
    cripto_yd(k) = theta_b(k) + pym(k);
    cripto_yd(k) = cripto_yd(k)*T;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Desencriptaci髇 de se馻les de trayectoria   
    decripto_xd(k) = cripto_xd(k)/T - theta_a(k);
    decripto_yd(k) = cripto_yd(k)/T - theta_b(k);
    
    decripto_xdp(k) = cripto_xd(k)/T - theta_ap(k);
    decripto_ydp(k) = cripto_yd(k)/T - theta_bp(k);
    
%###############################################################################
% Ley de control maestro
    pm = [decripto_xd(k);decripto_yd(k)]; %-------------------------> Vector de posici贸n del robot maestro
    
    nm = Hm(1:2,1);  n1m(k) = nm(1,1);  n2m(k) = nm(2,1);
    xi1m(k) = xim(1,1)/norm(xim,2); xi2m(k) = xim(2,1)/norm(xim,2);
    
    vm = norm(xim,2);
    
    xipm = -Lambda2m*xim - Lambda3m*(pm - pd) + ad + Lambda2m*vd;
    
    Wm = -(xim'*S*xipm)/(xim'*xim);
    %W = transpose(h)
    wm = Wm - (lambdam*nm'*S*xim)/norm(xim);
    Vm = [0 -wm vm;wm 0 0;0 0 0];
    xim = xim + tau*xipm; 
    
    Hm = Hm*expm(tau*Vm); %----------Resultado num茅rico de la matriz homog茅nea
    Hsm(:,:,k) = Hm; %---------------Variable auxiliar de almacenamiento de H
    Rpm(:,:,k) = Hsm(1:2,1:2,k); %------------Obtenci贸n y almacenamiento de R
    angm(:,:,k) = Rpm(:,1,k); %--------------Almacenamiento de los datos de R
    thetam(k) = atan2(angm(2,1,k),angm(1,1,k)); %-------Obtenci贸n del 谩ngulo 胃 

    vel_m = norm(xim,2)*Rpm(:,:,k)*e1; %--------------> Velocidad del robot maestro
%###############################################################################
% Ley de control esclavo
    h2p = -G1*h2 - G2*h1 - c*( b1*pm + ps );
    %h2p = -G1*h2 - G2*h1 + c*( pm + b1*ps ); % El valor de b1 determina la fase o antifase del sistema (el valor solo debe ser de -1 o 1)
    
    ns = Hs(1:2,1);  n1s(k) = ns(1,1);  n2s(k) = ns(2,1);
    xi1s(k) = xis(1,1)/norm(xis,2); xi2s(k) = xis(2,1)/norm(xis,2);
    
    vs = norm(xis,2);
    %vel_s = norm(xis,2)*Rs*e1;
    
    %xips = -Lambda2s*xis - Lambda3s*(-h1) + Lambda2s*h2 + h2p;
    xips = -Lambda2s*xis + Lambda3s*(h1) + Lambda2s*h2 + h2p;
    
    Ws = -(xis'*S*xips)/(xis'*xis);
    %W = transpose(h)
    ws = Ws - (lambdas*ns'*S*xis)/norm(xis);
    Vs = [0 -ws vs;ws 0 0;0 0 0];
    xis = xis + tau*xips; 
    
    Hs = Hs*expm(tau*Vs); %----------Resultado num茅rico de la matriz homog茅nea
    Hss(:,:,k) = Hs; %---------------Variable auxiliar de almacenamiento de H
    Rps(:,:,k) = Hss(1:2,1:2,k); %------------Obtenci贸n y almacenamiento de R
    angs(:,:,k) = Rps(:,1,k); %--------------Almacenamiento de los datos de R
    thetas(k) = atan2(angs(2,1,k),angs(1,1,k)); %-------Obtenci贸n del 谩ngulo 胃     
%###############################################################################
% Acoplamiento din坔mico
    h1 = h1 + tau*h2;
    h2 = h2 + tau*h2p;

    i = i + 1; %---------------------------Contador auxiliar para el tiempo
%###############################################################################
% Variables de almacenamiento 
    velo_M(k) = vm;
    velo_S(k) = vs;
%     velocidadx_m(k) = vel_m(1,:);
%     velocidady_m(k) = vel_m(2,:);
%     velocidadx_s(k) = vel_s(1,:);
%     velocidady_s(k) = vel_s(2,:);
    
    angular_m(k) = wm;
    angular_s(k) = ws;
    
    h1x(k) = h1(1,:);
    h1y(k) = h1(2,:);
    h2xp(k) = h2(1,:);
    h2yp(k) = h2(2,:);

end %end para MATLAB   
%endfor %endfor para OCTAVE
fuente = 14;
linea = 2;
%###############################################################################
% Graficacion de datos experimentales
figure()
subplot(3,1,1)
hold on
plot(t,pxm,'r','LineWidth',linea)
plot(t,pxs,'m','LineWidth',linea)
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
plot(t,pym,'Color',[1 0.2235 0],'LineWidth',linea)
plot(t,pys,'Color',[0.5058 0 1],'LineWidth',linea)
% plot(t,pym,'b','LineWidth',linea)
% plot(t,pys,'c','LineWidth',linea)
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
plot(t,thetam,'m','LineWidth',linea)
plot(t,thetas,'g','LineWidth',linea)
xlim([0 50])
ylim([-5 5])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('{\boldmath$R$}$(t)$ [rad]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('{\boldmath$R$}$_{m}(t)$','{\boldmath$R$}$_{s}(t)$','{\boldmath$R$}$_{d}(t)$')
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
plot(t,velo_M,'r','LineWidth',linea)
plot(t,velo_S,'b','LineWidth',linea)
% plot(t,velocidadx_s,'LineWidth',linea)
% plot(t,velocidady_s,'LineWidth',linea)
xlim([0 50])
ylim([-0.5 1])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\nu (t)$ [m/s]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$\nu_{m}(t)$','$\nu_{s}(t)$')
%LEG1 = legend('$\nu_{m_{x}}(t)$','$\nu_{m_{y}}(t)$','$\nu_{s_{x}}(t)$','$\nu_{s_{y}}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

subplot(2,1,2)
hold on
plot(t,angular_m,'m','LineWidth',linea)
plot(t,angular_s,'g','LineWidth',linea)
xlim([0 50])
ylim([-10 10])
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
%###############################################################################
% Graficacion del plano de fase 
figure()
hold on
plot(pxm,pym,'r','LineWidth',linea)
plot(pxs,pys,'b','LineWidth',linea)
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
%###############################################################################
% Graficacion del error del sistema
figure()
subplot(2,1,1)
hold on
plot(t,pxm-xd,'r','LineWidth',linea)
plot(t,pym-yd,'b','LineWidth',linea)
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
plot(t,pxm-pxs,'m','LineWidth',linea)
plot(t,pym-pys,'g','LineWidth',linea)
xlim([0 50])
ylim([-1 2])
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
%###############################################################################
% Graficacion del estado aumentado
figure()
hold on
plot(t,xi1m,'LineWidth',linea)
plot(t,xi2m,'LineWidth',linea)
plot(t,xi1s,'LineWidth',linea)
plot(t,xi2s,'LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\xi (t)$ [m/s]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$\xi_{m_{1}}(t)$','$\xi_{m_{2}}(t)$','$\xi_{s_{1}}(t)$','$\xi_{s_{2}}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)
%###############################################################################
% Graficacion de los estados del acoplamiento dinamico
figure()
subplot(2,1,1)
hold on
plot(t,h1x,'LineWidth',linea)
plot(t,h1y,'LineWidth',linea)
% plot(t,h2xp,'LineWidth',linea)
% plot(t,h2yp,'LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('{\boldmath$h$}$_{1} (t)$ [m]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$h_{1_{x}}(t)$','$h_{1_{y}}(t)$')
set(LEG1,'Interpreter','latex');
set(LEG1,'FontSize',fuente);
%set(LEG1,'Location','northeast')
set(LEG1,'Location','northeastoutside')
grid on
set(gca,'fontsize',fuente)

subplot(2,1,2)
hold on
% plot(t,h1x,'LineWidth',linea)
% plot(t,h1y,'LineWidth',linea)
plot(t,h2xp,'LineWidth',linea)
plot(t,h2yp,'LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('{\boldmath$h$}$_{2} (t)$ [m/s]','Interpreter','latex','fontsize',fuente)
LEG1 = legend('$h_{2_{x}}(t)$','$h_{2_{y}}(t)$')
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
plot(t,pxm,'LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$x_{m}(t)$ [m]','Interpreter','latex','fontsize',fuente)
grid on
set(gca,'fontsize',fuente)

subplot(4,1,2)
plot(t,cripto_xd,'LineWidth',linea)
xlim([0 50])
ylim([-2.5 2.5])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\check{x}_{m}(t)$ [m]','Interpreter','latex','fontsize',fuente)
grid on
set(gca,'fontsize',fuente)

subplot(4,1,3)
plot(t,decripto_xd,'LineWidth',linea)
xlim([0 50])
ylim([-1.2 1.2])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\hat{x}_{m}(t)$ [m]','Interpreter','latex','fontsize',fuente)
grid on
set(gca,'fontsize',fuente)

subplot(4,1,4)
plot(t,decripto_xdp,'LineWidth',linea)
xlim([0 50])
ylim([-5000 5000])
set(groot,'defaultAxesTickLabelInterpreter','latex'); 
xticklabels(strrep(xticklabels,'-','$$-$$'));
yticklabels(strrep(yticklabels,'-','$$-$$'));
xlabel('$t$ [s]','Interpreter','latex','fontsize',fuente)
ylabel('$\breve{x}_{m}(t)$ [m]','Interpreter','latex','fontsize',fuente)
grid on
set(gca,'fontsize',fuente)

