%% PARTE 1
clc
sim("Control_Cinematico.mdl");
figure(1)
plot3(xr, yr, zr, 'LineWidth', 2)
grid on
xlabel('Eje X')
ylabel('Eje Y')
zlabel('Eje Z')
title('Trayectoria a seguir')

%--------------------------Graficas posicion------------------------------%

figure(2)
ej1=subplot(3,1,1);
plot(tout,qr(:,1));
grid on;
title('qr1');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej2=subplot(3,1,2);
plot(tout,qr(:,2));
grid on;
title('qr2');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

ej3=subplot(3,1,3);
plot(tout,qr(:,3));
grid on;
title('qr3');
ylabel('posición(m)');
 xlabel('Tiempo (s)'); 

 linkaxes([ej1 ej2 ej3], 'x');

 %----------------------Graficas velocidad--------------------------------%

 figure(3)
ej12=subplot(3,1,1);
plot(tout,qdr(:,1));
grid on;
title('qdr1');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej22=subplot(3,1,2);
plot(tout,qdr(:,2));
grid on;
title('qdr2');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

ej32=subplot(3,1,3);
plot(tout,qdr(:,3));
grid on;
title('qdr3');
ylabel('velocidad m/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej12 ej22 ej32], 'x');

  %--------------------------Graficas aceleración-------------------------%
  figure(4)
ej13=subplot(3,1,1);
plot(tout,qddr(:,1));
grid on;
title('qdd1');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej23=subplot(3,1,2);
plot(tout,qddr(:,2));
grid on;
title('qddr2');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

ej33=subplot(3,1,3);
plot(tout,qddr(:,3));
grid on;
title('qddr3');
ylabel('aceleración m^2/s');
 xlabel('Tiempo (s)'); 

 linkaxes([ej13 ej23 ej33], 'x');