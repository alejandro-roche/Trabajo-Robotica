%% PARTE 1
clc
clear all

syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms q1 q2 q3 q4 q5 q6 real 
syms phi theta psi real
syms nx ox ax x ny oy ay y nz oz az z
PI = sym(pi);


% DATOS CINEMÁTICOS DEL BRAZO DEL ROBOT
% Dimensiones (m)
    L0=1;
    L1=1.5;
    L2A=0.5;
    L2B=1.5;
    L3=1;
    rint=0.08;
    rext=0.1;
    rho=5800;
    Ar=pi*(rext^2-rint^2);
    rhoL=rho*Ar;


% DATOS DINÁMICOS DEL BRAZO DEL ROBOT

%----------------------------Eslabón 0------------------------------------% 
% (fijo. No hace falta especificarlo porque no se mueve respecto al sistema Base B)
  
  %Masa eslabón 0
  m0= 65.5965; % kg
  
  %Centro de masas eslabón 0
  s00 = [0,-L0/2,0]' % m
  
  %Tensor de Inercias
  I00=zeros(3); % kg.m2


%----------------------------Eslabón 1------------------------------------%
  
  %Masa eslabón 1
  m1= 98.3947; % kg
  
  %Centro de masas eslabón 1
  s11 = [ 0, L1/2, 0]' % m
  
  %Momentos de Inercia
  I1xx= 1/12 * m1 * L1^2;
  I1yy= 1/2 * m1 *(rext^2+rint^2);
  I1zz= 1/12 * m1 * L1^2;
  
  %Tensor de Inercias
  I11=[ I1xx, 0 , 0 ; 0, I1yy , 0; 0, 0, I1zz] % kg.m2

%----------------------------Eslabón 2------------------------------------%
  
  %Masa eslabón 2
  m2= 131.193; % kg
  mA=(m2*L2A)/(L2A+L2B) ;%Masa barra A
  mB=(m2*L2B)/(L2A+L2B) ;%Masa barra B
  
  sA2=[-L2A/2,L2B,0]';%centro de masas barra A
  sB2=[0,L2B/2,0]'   ;%centro de masas barra B
 
  %Centro de masas eslabón 2
  s22 = (mA*sA2+mB*sB2)/(mA+mB)
  
  %Posicion del centro de masas de cada barra respecto a la del eslabón
  r2A=s22-sA2;
  r2B=s22-sB2;

  %Momentos de Inercia de barra A
  I2Axx= 1/2 * mA *(rext^2+rint^2);
  I2Ayy= 1/12 * mB * L2A^2;
  I2Azz= 1/12 * mB * L2A^2;
  
  %Tensor de Inercias barra A
  I2A=[ I2Axx, 0 , 0 ; 0, I2Ayy , 0; 0, 0, I2Azz];

  %Teorema Steiner
  Steiner2A=norm(r2A)^2*eye(3)-r2A*r2A';
  
  %Aportacion que hace la barra A al tensor de Inercias del eslabón 2
  I2Acdm=I2A+mA*Steiner2A ;

  %Momentos de Inercia de barra B
  I2Bxx=1/12 * mB * L2B^2;
  I2Byy=1/2 * mB *(rext^2+rint^2);
  I2Bzz=1/12 * mB * L2B^2;

  %Tensor de Inercias barra B
  I2B=[ I2Bxx, 0 , 0 ; 0, I2Byy , 0; 0, 0, I2Bzz];

  %Teorema Steiner
  Steiner2B=norm(r2B)^2*eye(3)-r2B*r2B';

  %Aportacion que hace la barra B al tensor de Inercias del eslabón 2
  I2Bcdm=I2B+mB*Steiner2B ;
  
  %Tensor de Inercias
  I22=I2Acdm+I2Bcdm % kg.m2

%-----------------------------Eslabón 3-----------------------------------%
  
  %Masa del eslabón 3
  m3=65.5965 ; % kg

  %Centro de masas eslabón 3
  s33 = [ 0,  0,-L3/2 ]' % m
  
  %Momentos de Inercia
  I3xx=1/12 * m3 * L3^2;;
  I3yy=1/12 * m3 * L3^2;;
  I3zz=1/2 * m3 *(rext^2+rint^2);

  %Tensor de Inercias
  I33=[ I3xx, 0 , 0 ; 0, I3yy , 0;0 ,0 ,I3zz ] % kg.m2
 

%% PARTE 3
clc

syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms q1 q2 q3 q4 q5 q6 real 
syms phi theta psi real
syms nx ox ax x ny oy ay y nz oz az z
PI = sym(pi);

sim("Simulador_Dinamica.slx");

%--------------------------Graficas posicion------------------------------%

figure(1)
ej1=subplot(3,1,1);
plot(tout,q_proof(:,1),'LineWidth', 1.5);
hold on;
plot(tout,q(:,1));
legend('q_proof1','q1');
grid on;
title('q1');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej2=subplot(3,1,2);
plot(tout,q_proof(:,2),'LineWidth', 1.5);
hold on;
plot(tout,q(:,2));
legend('q_proof2','q2');
grid on;
title('q2');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej3=subplot(3,1,3);
plot(tout,q_proof(:,3),'LineWidth', 1.5);
hold on;
plot(tout,q(:,3));
legend('q_proof3','q3');
grid on;
title('q3');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

 linkaxes([ej1 ej2 ej3], 'x');

 %----------------------Graficas velocidad--------------------------------%

 figure(2)
ej12=subplot(3,1,1);
plot(tout,qd_proof(:,1),'LineWidth', 1.5);
hold on;
plot(tout,qd(:,1));
legend('qd_proof1','qd1');
grid on;
title('qd1');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej22=subplot(3,1,2);
plot(tout,qd_proof(:,2),'LineWidth', 1.5);
hold on;
plot(tout,qd(:,2));
legend('qd_proof2','qd2');
grid on;
title('qd2');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej32=subplot(3,1,3);
plot(tout,qd_proof(:,3),'LineWidth', 1.5);
hold on;
plot(tout,qd(:,3));
legend('qd_proof3','qd3');
grid on;
title('qd3');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

  %--------------------------Graficas aceleración-------------------------%
  figure(3)
ej13=subplot(3,1,1);
plot(tout,qdd_proof(:,1),'LineWidth', 1.5);
hold on;
plot(tout,qdd(:,1));
legend('qdd_proof1','qdd1');
grid on;
title('qdd1');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej23=subplot(3,1,2);
plot(tout,qdd_proof(:,2),'LineWidth', 1.5);
hold on;
plot(tout,qdd(:,2));
legend('qdd_proof2','qdd2');
grid on;
title('qdd2');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej33=subplot(3,1,3);
plot(tout,qdd_proof(:,3),'LineWidth', 1.5);
hold on;
plot(tout,qdd(:,3));
legend('qdd_proof3','qdd3');
grid on;
title('qdd3');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej13 ej23 ej33], 'x');


 %------------------------------ERRORES-----------------------------------%

 %posiciones
errorq1=q_proof(:,1)-q(:,1);
errorq2=q_proof(:,2)-q(:,2);
errorq3=q_proof(:,3)-q(:,3);

%velocidades
errorqd1=qd_proof(:,1)-qd(:,1);
errorqd2=qd_proof(:,2)-qd(:,2);
errorqd3=qd_proof(:,3)-qd(:,3);

%aceleraciones
errorqdd1=qdd_proof(:,1)-qdd(:,1);
errorqdd2=qdd_proof(:,2)-qdd(:,2);
errorqdd3=qdd_proof(:,3)-qdd(:,3);

%--------------------------Graficas posicion------------------------------%

figure(4)
ej1=subplot(3,1,1);
plot(tout,errorq1,'LineWidth', 1.5);
grid on;
title('error q1');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej2=subplot(3,1,2);
plot(tout,errorq2,'LineWidth', 1.5);
grid on;
title('error q2');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej3=subplot(3,1,3);
plot(tout,errorq3,'LineWidth', 1.5);
grid on;
title('error q3');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

 linkaxes([ej1 ej2 ej3], 'x');

 %----------------------Graficas velocidad--------------------------------%

 figure(5)
ej12=subplot(3,1,1);
plot(tout,errorqd1,'LineWidth', 1.5);
grid on;
title('error qd1');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej22=subplot(3,1,2);
plot(tout,errorqd2,'LineWidth', 1.5);
grid on;
title('error qd2');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej32=subplot(3,1,3);
plot(tout,errorqd3,'LineWidth', 1.5);
grid on;
title('error qd3');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

  %--------------------------Graficas aceleración-------------------------%
  figure(6)
ej13=subplot(3,1,1);
plot(tout,errorqdd1,'LineWidth', 1.5);
grid on;
title('error qdd1');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej23=subplot(3,1,2);
plot(tout,errorqdd2,'LineWidth', 1.5);
grid on;
title('error qdd2');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej33=subplot(3,1,3);
plot(tout,errorqdd3,'LineWidth', 1.5);
grid on;
title('error qdd3');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej13 ej23 ej33], 'x');
