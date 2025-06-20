%% PARTE D
clc

syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms q1 q2 q3 qd1 qd2 qd3 qdd1 qdd2 qdd3 real 
syms phi theta psi real
syms nx ox ax x ny oy ay y nz oz az z
PI = sym(pi);
g=9.8;

%----------------------Linealizacion del modelo---------------------------%

%Mattriz de Inercias
Ma=[32.798*q3 + 11.751*cos(2.0*q2) - 16.399*sin(2.0*q2) - 32.798*q3*cos(2.0*q2) - 32.798*q3*sin(2.0*q2) - 32.798*q3^2*cos(2.0*q2) + 32.798*q3^2 + 257.66, 49.197*cos(q2) + 86.095*sin(q2) + 98.395*q3*cos(q2), 98.395*sin(q2);
    86.095*sin(q2) + 98.395*cos(q2)*(q3 + 0.5),                    65.596*q3^2 + 65.596*q3 + 69.625,         32.798;
    98.395*sin(q2),                                              32.798,         69.863];
 
q2_vals = linspace(-pi, pi, 200);
q3_vals = linspace(0, 1.5, 200);

max_norma = -Inf;

for q2 = q2_vals
    for q3 = q3_vals
       Ma=[32.798*q3 + 11.751*cos(2.0*q2) - 16.399*sin(2.0*q2) - 32.798*q3*cos(2.0*q2) - 32.798*q3*sin(2.0*q2) - 32.798*q3^2*cos(2.0*q2) + 32.798*q3^2 + 257.66, 49.197*cos(q2) + 86.095*sin(q2) + 98.395*q3*cos(q2), 98.395*sin(q2);
    86.095*sin(q2) + 98.395*cos(q2)*(q3 + 0.5),                    65.596*q3^2 + 65.596*q3 + 69.625,         32.798;
    98.395*sin(q2),                                              32.798,         69.863];
 
        norma_actual = trace(Ma);  % o norm(Ma, 2)

        if norma_actual > max_norma
            max_norma = norma_actual;
            q2_max = q2;
            q3_max = q3;
        end
    end
end

fprintf("Máxima inercia (traza): %.3f en q2 = %.3f rad, q3 = %.3f m\n", max_norma, q2_max, q3_max)
%Despreciamos terminos no diagonales-->Desacoplamos el sistema
 %Llamaremos a los terminos diagonales de la matriz de inercias: ai
 

%Matriz de aceleraciones centripedas y Coriolis

Va=[0.040639*qd1 + 86.095*qd2^2*cos(q2) - 49.197*qd2^2*sin(q2) + 32.798*qd1*qd3 + 196.79*qd2*qd3*cos(q2) - 98.395*q3*qd2^2*sin(q2) + 65.596*q3*qd1*qd3 - 32.798*qd1*qd2*cos(2.0*q2) - 32.798*qd1*qd3*cos(2.0*q2) - 23.501*qd1*qd2*sin(2.0*q2) - 32.798*qd1*qd3*sin(2.0*q2) - 65.596*q3*qd1*qd2*cos(2.0*q2) - 65.596*q3*qd1*qd3*cos(2.0*q2) + 65.596*q3*qd1*qd2*sin(2.0*q2) + 65.596*q3^2*qd1*qd2*sin(2.0*q2);
    0.1412*qd2 + 65.596*qd2*qd3 + 16.399*qd1^2*cos(2.0*q2) + 11.751*qd1^2*sin(2.0*q2) + 32.798*q3*qd1^2*cos(2.0*q2) - 32.798*q3*qd1^2*sin(2.0*q2) - 32.798*q3^2*qd1^2*sin(2.0*q2) + 131.19*q3*qd2*qd3;
    0.18402*qd3 - 32.798*qd1^2*sin(q2)^2 - 65.596*q3*qd2^2 + 16.399*qd1^2*sin(2.0*q2) - 32.798*qd2^2 - 65.596*q3*qd1^2*sin(q2)^2];
 
     
%Se desprecia termino de Coriolis y las velocidadaes y caeleraciones estan en equilibrio
 %llamaremos a los términos lineales de Va : bi
 

%Matriz del par gravitatorio

Ga=[-1.0*g*(172.19*sin(q1) - 90.195*cos(q1)*cos(q2) + 32.798*cos(q1)*sin(q2) + 65.596*q3*cos(q1)*sin(q2));
    -8.1996*g*sin(q1)*(4.0*cos(q2) + 11.0*sin(q2) + 8.0*q3*cos(q2));
     -65.596*g*sin(q1)*sin(q2)];
       
%------------------------Funcion de transferencia-------------------------%

%                 1
%     G(s)=-----------------
%           s*(ai*s + bi)

simplify(subs(Ma, {q2, q3}, {-1.31, 1.5}))%Maximizar valores articulares

Mamax=[37027565893845/137438953472 0 0;
    0 31561/100 0;
    0 0 69863/1000];
Vamax=[0.040639;0.1412; 0.18402];

G11=tf(1,conv([1 0],[37027565893845/137438953472 0.040639]))
G22=tf(1,conv([1 0],[31561/100 0.1412]))
G33=tf(1, conv([1 0],[ 69863/1000 0.18402]))

%% PD descentralizado sin cancelacion de dinamica sin compensacion de gravedad

clc
clear all
%Funciones de transferencia

G11=tf(1,conv([1 0],[37027565893845/137438953472 0.040639]));
G22=tf(1,conv([1 0],[31561/100 0.1412]));
G33=tf(1, conv([1 0],[ 69863/1000 0.18402]));

 %Tiempo de subida en bucle cerrado
tsbc=0.1;
taubc=0.1/3;


 %lugar de las raices donde quiero que caigan los polos para cumplir el tiempo de subida
pbc=-1/taubc;

 %PD
 %              
 %    C(s)= Kp*(Td*s+1)
 %                     


%-----------------Funcion de transferencia primer motor-------------------%

% rltool(G11)

 %Controlador obtenido
 Cs1=tf(16242*[1 15.07], [0 1]);

 %Parametros del controlador

 Td1=0.0663;
 Kp1=2.448e5 ;
 Kd1=Kp1*Td1;

 %---------------Funcion de transferencia segundo motor-------------------%

 %rltool(G22)

  %Controlador obtenido
 Cs2=tf(18936*[1 15], [0 1]);

  %Parametros del controlador
 
 Td2=0.067;
 Kp2=284040;
 Kd2=Kp2*Td2;

  %---------------Funcion de transferencia tercer motor-------------------%

%rltool(G33);

  %Controlador obtenido
 Cs3=tf(4177.6*[1 14.95], [0 1]);

  %Parametros del controlador
 Td3=0.067;
 Kp3= 6.24e4;
 Kd3=Kp3*Td3;

 Kd=[Kd1;Kd2;Kd3];
 Kp=[Kp1;Kp2;Kp3];

 %--------------------Evolución coordenadas articulares--------------------%

%sim("Simulador_PD_sincompensacion.slx");
sim("Simulador_PD_sincompensacion_Robustez.slx");%%Analisis de robustez
 %Graficas posicion%
figure(2)
ej1=subplot(3,1,1);
plot(t,qr(:,1),'LineWidth', 1.5);
hold on;
plot(t,q(:,1));
legend('qr1','q1');
grid on;
title('q1');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej2=subplot(3,1,2);
plot(t,qr(:,2),'LineWidth', 1.5);
hold on;
plot(t,q(:,2));
legend('qr2','q2');
grid on;
title('q2');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej3=subplot(3,1,3);
plot(t,qr(:,3),'LineWidth', 1.5);
hold on;
plot(t,q(:,3));
legend('qr3','q3');
grid on;
title('q3');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

 linkaxes([ej1 ej2 ej3], 'x');

 %Graficas velocidad%
figure(3)
ej12=subplot(3,1,1);
plot(t,qdr(:,1),'LineWidth', 1.5);
hold on;
plot(t,qd(:,1));
legend('qdr1','qd1');
grid on;
title('qd1');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej22=subplot(3,1,2);
plot(t,qdr(:,2),'LineWidth', 1.5);
hold on;
plot(t,qd(:,2));
legend('qdr2','qd2');
grid on;
title('qd2');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej32=subplot(3,1,3);
plot(t,qdr(:,3),'LineWidth', 1.5);
hold on;
plot(t,qd(:,3));
legend('qdr3','qd3');
grid on;
title('qd3');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

 %Graficas aceleraciones%
  figure(1)
ej13=subplot(3,1,1);
plot(t,qddr(:,1),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,1));
grid on;
title('qdd1');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej23=subplot(3,1,2);
plot(t,qddr(:,2),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,2));
grid on;
title('qddr2');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej33=subplot(3,1,3);
plot(t,qddr(:,3),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,3));
grid on;
title('qddr3');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej13 ej23 ej33], 'x');

%-------------------------Trayectoria a seguir----------------------------%

figure(4)
plot3(xr, yr, zr, 'LineWidth', 2);
hold on
plot3(x, y, z )
grid on
legend('Referencia','Trayectoria real')
xlabel('Eje X')
ylabel('Eje Y')
zlabel('Eje Z')
title('Trayectoria a seguir')


figure(5)
ejx=subplot(3,1,1);
plot(t,xr,'LineWidth', 1.5);
hold on;
plot(t,x);
legend('xr','x');
grid on;
title('x');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

ejy=subplot(3,1,2);
plot(t,yr,'LineWidth', 1.5);
hold on;
plot(t,y);
legend('yr','y');
grid on;
title('y');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

ejz=subplot(3,1,3);
plot(t,zr,'LineWidth', 1.5);
hold on;
plot(t,z);
legend('zr','z');
grid on;
title('z');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');
%% PD descentralizado sin cancelacion de dinamica con compensacion de gravedad
clc
clear all
%Funciones de transferencia

G11=tf(1,conv([1 0],[37027565893845/137438953472 0.040639]));
G22=tf(1,conv([1 0],[31561/100 0.1412]));
G33=tf(1, conv([1 0],[ 69863/1000 0.18402]));

 %Tiempo de subida en bucle cerrado
tsbc=0.1;
taubc=0.1/3;


 %lugar de las raices donde quiero que caigan los polos para cumplir el tiempo de subida
pbc=-1/taubc;

 %PD
 %              
 %    C(s)= Kp*(Td*s+1)
 %                     


%-----------------Funcion de transferencia primer motor-------------------%

% rltool(G11)

 %Controlador obtenido
 Cs1=tf(16242*[1 15.07], [0 1]);

 %Parametros del controlador

 Td1=0.0663;
 Kp1=2.448e5 ;
 Kd1=Kp1*Td1;

 %---------------Funcion de transferencia segundo motor-------------------%

 %rltool(G22)

  %Controlador obtenido
 Cs2=tf(18936*[1 15], [0 1]);

  %Parametros del controlador
 
 Td2=0.067;
 Kp2=284040;
 Kd2=Kp2*Td2;

  %---------------Funcion de transferencia tercer motor-------------------%

%rltool(G33);

  %Controlador obtenido
 Cs3=tf(4177.6*[1 14.95], [0 1]);

  %Parametros del controlador
 Td3=0.067;
 Kp3= 6.24e4;
 Kd3=Kp3*Td3;

 Kd=[Kd1;Kd2;Kd3];
 Kp=[Kp1;Kp2;Kp3];


 %--------------------Evolución coordenadas articulares--------------------%

%sim("Simulador_PD_concompensacion.slx");
sim("Simulador_PD_concompensacion_Robustez.slx");%Analisis de Robustez
 %Graficas posicion%
figure(2)
ej1=subplot(3,1,1);
plot(t,qr(:,1),'LineWidth', 1.5);
hold on;
plot(t,q(:,1));
legend('qr1','q1');
grid on;
title('q1');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej2=subplot(3,1,2);
plot(t,qr(:,2),'LineWidth', 1.5);
hold on;
plot(t,q(:,2));
legend('qr2','q2');
grid on;
title('q2');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej3=subplot(3,1,3);
plot(t,qr(:,3),'LineWidth', 1.5);
hold on;
plot(t,q(:,3));
legend('qr3','q3');
grid on;
title('q3');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

 linkaxes([ej1 ej2 ej3], 'x');

 %Graficas velocidad%
figure(3)
ej12=subplot(3,1,1);
plot(t,qdr(:,1),'LineWidth', 1.5);
hold on;
plot(t,qd(:,1));
legend('qdr1','qd1');
grid on;
title('qd1');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej22=subplot(3,1,2);
plot(t,qdr(:,2),'LineWidth', 1.5);
hold on;
plot(t,qd(:,2));
legend('qdr2','qd2');
grid on;
title('qd2');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej32=subplot(3,1,3);
plot(t,qdr(:,3),'LineWidth', 1.5);
hold on;
plot(t,qd(:,3));
legend('qdr3','qd3');
grid on;
title('qd3');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

 %Graficas aceleraciones%
  figure(1)
ej13=subplot(3,1,1);
plot(t,qddr(:,1),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,1));
grid on;
title('qdd1');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej23=subplot(3,1,2);
plot(t,qddr(:,2),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,2));
grid on;
title('qddr2');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej33=subplot(3,1,3);
plot(t,qddr(:,3),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,3));
grid on;
title('qddr3');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej13 ej23 ej33], 'x');

%-------------------------Trayectoria a seguir----------------------------%

figure(4)
plot3(xr, yr, zr, 'LineWidth', 2);
hold on
plot3(x, y, z )
grid on
legend('Referencia','Trayectoria real')
xlabel('Eje X')
ylabel('Eje Y')
zlabel('Eje Z')
title('Trayectoria a seguir')


figure(5)
ejx=subplot(3,1,1);
plot(t,xr,'LineWidth', 1.5);
hold on;
plot(t,x);
legend('xr','x');
grid on;
title('x');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

ejy=subplot(3,1,2);
plot(t,yr,'LineWidth', 1.5);
hold on;
plot(t,y);
legend('yr','y');
grid on;
title('y');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

ejz=subplot(3,1,3);
plot(t,zr,'LineWidth', 1.5);
hold on;
plot(t,z);
legend('zr','z');
grid on;
title('z');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');


 %% PID descentralizado sin cancelacion de dinamica
clc
clear all
%Funciones de transferencia

G11=tf(1,conv([1 0],[37027565893845/137438953472 0.040639]));
G22=tf(1,conv([1 0],[31561/100 0.1412]));
G33=tf(1, conv([1 0],[ 69863/1000 0.18402]));

 %Tiempo de subida en bucle cerrado
tsbc=0.1;
taubc=0.1/3;
taubceq=taubc/2;

 %lugar de las raices donde quiero que caigan los polos para cumplir el tiempo de subida
pbc=-1/taubceq;

 %PID
 %              (Ti*Td*s + Ti*s + 1)
 %    C(s)= Kp*----------------------
 %                     Ti*s


%-----------------Funcion de transferencia primer motor-------------------%

 %rltool(G11)

 %Controlador obtenido
 Cs1=tf(36679*conv([1 20.17], [1 20.17]), [1 0]);

 %Parametros del controlador
 Ti1=0.1;
 Td1=0.00246/Ti1;
 Kp1=1.492e7 * Ti1;
 Ki1=Kp1/Ti1;
 Kd1=Kp1*Td1;

 %---------------Funcion de transferencia segundo motor-------------------%

 %rltool(G22)

  %Controlador obtenido
 Cs2=tf(42607*conv([1 20], [1 20]), [1 0]);

  %Parametros del controlador
 Ti2=0.1;
 Td2=0.0025/Ti2;
 Kp2=1.704e7 * Ti2;
 Ki2=Kp2/Ti2;
 Kd2=Kp2*Td2;

  %---------------Funcion de transferencia tercer motor-------------------%

% rltool(G33);

  %Controlador obtenido
 Cs3=tf(9431.1*conv([1 20], [1 20]), [1 0]);

  %Parametros del controlador
 Ti3=0.1;
 Td3=0.0025/Ti3;
 Kp3= 3.772e6* Ti3;
 Ki3=Kp3/Ti3;
 Kd3=Kp3*Td3;

 Kd=[Kd1;Kd2;Kd3];
 Kp=[Kp1;Kp2;Kp3];
 Ki=[Ki1;Ki2;Ki3];

 %--------------------Evolución coordenadas articulares--------------------%

%sim("Simulador_PID.slx");
sim("Simulador_PID_Robustez.slx");%Analisis de robustez
 %Graficas posicion%
figure(2)
ej1=subplot(3,1,1);
plot(t,qr(:,1),'LineWidth', 1.5);
hold on;
plot(t,q(:,1));
legend('qr1','q1');
grid on;
title('q1');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej2=subplot(3,1,2);
plot(t,qr(:,2),'LineWidth', 1.5);
hold on;
plot(t,q(:,2));
legend('qr2','q2');
grid on;
title('q2');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej3=subplot(3,1,3);
plot(t,qr(:,3),'LineWidth', 1.5);
hold on;
plot(t,q(:,3));
legend('qr3','q3');
grid on;
title('q3');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

 linkaxes([ej1 ej2 ej3], 'x');

 %Graficas velocidad%
figure(3)
ej12=subplot(3,1,1);
plot(t,qdr(:,1),'LineWidth', 1.5);
hold on;
plot(t,qd(:,1));
legend('qdr1','qd1');
grid on;
title('qd1');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej22=subplot(3,1,2);
plot(t,qdr(:,2),'LineWidth', 1.5);
hold on;
plot(t,qd(:,2));
legend('qdr2','qd2');
grid on;
title('qd2');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej32=subplot(3,1,3);
plot(t,qdr(:,3),'LineWidth', 1.5);
hold on;
plot(t,qd(:,3));
legend('qdr3','qd3');
grid on;
title('qd3');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

  %Graficas aceleraciones%
  figure(1)
ej13=subplot(3,1,1);
plot(t,qddr(:,1),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,1));
grid on;
title('qdd1');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej23=subplot(3,1,2);
plot(t,qddr(:,2),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,2));
grid on;
title('qddr2');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej33=subplot(3,1,3);
plot(t,qddr(:,3),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,3));
grid on;
title('qddr3');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej13 ej23 ej33], 'x');

%-------------------------Trayectoria a seguir----------------------------%

figure(4)
plot3(xr, yr, zr, 'LineWidth', 2);
hold on
plot3(x, y, z )
grid on
legend('Referencia','Trayectoria real')
xlabel('Eje X')
ylabel('Eje Y')
zlabel('Eje Z')
title('Trayectoria a seguir')


figure(5)
ejx=subplot(3,1,1);
plot(t,xr,'LineWidth', 1.5);
hold on;
plot(t,x);
legend('xr','x');
grid on;
title('x');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

ejy=subplot(3,1,2);
plot(t,yr,'LineWidth', 1.5);
hold on;
plot(t,y);
legend('yr','y');
grid on;
title('y');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

ejz=subplot(3,1,3);
plot(t,zr,'LineWidth', 1.5);
hold on;
plot(t,z);
legend('zr','z');
grid on;
title('z');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

 %% Controlador de par calculado
clc
clear all
  

 %Tiempo de subida en bucle cerrado
tsbc=0.1;

wn=3/tsbc;

Kp1=wn^2;%Con ganancia derivativa C=1
Kd1=2*wn;

 Kd=[Kd1;Kd1;Kd1];
 Kp=[Kp1;Kp1;Kp1];


 %--------------------Evolución coordenadas articulares--------------------%

%sim("Simulador_Control_Par_calculado.slx");
sim("Simulador_Control_Par_calculado_Robustez.slx");%Analisis de Robustez
 %Graficas posicion%
figure(2)
ej1=subplot(3,1,1);
plot(t,qr(:,1),'LineWidth', 1.5);
hold on;
plot(t,q(:,1));
legend('qr1','q1');
grid on;
title('q1');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej2=subplot(3,1,2);
plot(t,qr(:,2),'LineWidth', 1.5);
hold on;
plot(t,q(:,2));
legend('qr2','q2');
grid on;
title('q2');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej3=subplot(3,1,3);
plot(t,qr(:,3),'LineWidth', 1.5);
hold on;
plot(t,q(:,3));
legend('qr3','q3');
grid on;
title('q3');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

 linkaxes([ej1 ej2 ej3], 'x');

 %Graficas velocidad%
figure(3)
ej12=subplot(3,1,1);
plot(t,qdr(:,1),'LineWidth', 1.5);
hold on;
plot(t,qd(:,1));
legend('qdr1','qd1');
grid on;
title('qd1');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej22=subplot(3,1,2);
plot(t,qdr(:,2),'LineWidth', 1.5);
hold on;
plot(t,qd(:,2));
legend('qdr2','qd2');
grid on;
title('qd2');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej32=subplot(3,1,3);
plot(t,qdr(:,3),'LineWidth', 1.5);
hold on;
plot(t,qd(:,3));
legend('qdr3','qd3');
grid on;
title('qd3');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

   %Graficas aceleraciones%
  figure(1)
ej13=subplot(3,1,1);
plot(t,qddr(:,1),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,1));
grid on;
title('qdd1');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej23=subplot(3,1,2);
plot(t,qddr(:,2),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,2));
grid on;
title('qddr2');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej33=subplot(3,1,3);
plot(t,qddr(:,3),'LineWidth', 1.5);
hold on;
plot(t,qdd(:,3));
grid on;
title('qddr3');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej13 ej23 ej33], 'x');

%-------------------------Trayectoria a seguir----------------------------%

figure(4)
plot3(xr, yr, zr, 'LineWidth', 2);
hold on
plot3(x, y, z )
grid on
legend('Referencia','Trayectoria real')
xlabel('Eje X')
ylabel('Eje Y')
zlabel('Eje Z')
title('Trayectoria a seguir')


figure(5)
ejx=subplot(3,1,1);
plot(t,xr,'LineWidth', 1.5);
hold on;
plot(t,x);
legend('xr','x');
grid on;
title('x');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

ejy=subplot(3,1,2);
plot(t,yr,'LineWidth', 1.5);
hold on;
plot(t,y);
legend('yr','y');
grid on;
title('y');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

ejz=subplot(3,1,3);
plot(t,zr,'LineWidth', 1.5);
hold on;
plot(t,z);
legend('zr','z');
grid on;
title('z');
ylabel('posicion m');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

