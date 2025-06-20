%% PARTE 2
clc
clear all
syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms q1 q2 q3 q4 q5 q6 real  
PI = sym(pi);

%-------------Cinematica Directa-------------------%

 %Robot 6gdl
AB0=simplify(Aij(PI/2,L0,0,PI/2))                 %Matriz de transformacion
A01=simplify(Aij(q1,L1,0,-PI/2))
A12=simplify(Aij(q2,L2B,L2A,-PI/2))
A23=simplify(Aij(-PI/2,L3+q3,0,PI/2))
A34=simplify(Aij(0,L4+q4,0,PI/2))
A45=simplify(Aij(q5,0,-L5,PI))
A56=simplify(Aij(0,L6+q6,0,0))

 %Posicion HOME
TB6=simplify(AB0*A01*A12*A23*A34*A45*A56);       %Matriz de tarea
subs(TB6,[q1,q2,q3,q4,q5,q6],[0,0,0,0,0,0])


%% PARTE 3
clc
clear all

syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms q1 q2 q3 q4 q5 q6 real 
syms phi theta psi real
PI = sym(pi);

 %-----------------------------Robot 3gd----------------------------------%
AB0=simplify(Aij(PI/2,L0,0,PI/2))
A01=simplify(Aij(q1,L1,0,-PI/2))
A12=simplify(Aij(q2,L2B,L2A,-PI/2))
A23=simplify(Aij(0,L3+q3,0,0))

 %Posicion HOME
TB3=simplify(AB0*A01*A12*A23)
subs(TB3,[q1,q2,q3],[0,0,0])

 %Ecuaciones de posicion
x = L1 - cos(q2)*(L3 + q3) - L2A*sin(q2);                                
y = L2A*cos(q1)*cos(q2) - cos(q1)*sin(q2)*(L3 + q3) - L2B*sin(q1);       
z = L0 + L2B*cos(q1) - sin(q1)*sin(q2)*(L3 + q3) + L2A*cos(q2)*sin(q1);

 %Angulos de Euler
ZXZ=rotaz(phi)*rotax(theta)*rotaz(psi);     %Matriz de rotacion
TB3=TB3(1:3,1:3)                            %Matriz de orientacion
ZXZ=ZXZ(1:3,1:3)

phi=atan2(-cos(q2),cos(q1)*sin(q2));        %R13/R23-->tan(phi)
psi=atan2(cos(q2)*sin(q1),-cos(q1));        %R31/R32-->tan(psi)
theta=atan2(cos(q2),-sin(psi)*sin(q2));     %R31/R33-->tan(theta)

%% PARTE 4
clc
clear all

syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms q1 q2 q3 q4 q5 q6 real 
syms phi theta psi real
syms nx ox ax x ny oy ay y nz oz az z
PI = sym(pi);

%---------------------Articulaciones Robot--------------------------------%
  L0 = 1; 
  L1=1.5;
  L2A=0.5;
  L2B=1.5;
  L3=1;

%---------------------Cinematica inversa----------------------------------%

 %Matriz simbolica-->AB0*A01*A12*A23=Td
Td=[nx ox ax x
    ny oy ay y
    nz oz az z
    0 0 0 1];  
 
%Robot 3gdl
AB0=simplify(Aij(PI/2,L0,0,PI/2));
A01=simplify(Aij(q1,L1,0,-PI/2));
A12=simplify(Aij(q2,L2B,L2A,-PI/2));
A23=simplify(Aij(0,L3+q3,0,0));

%A12*A23= A01-1 * AB0-1 * Td
izq=simplify(A12*A23)     
der=simplify(inv(A01)*inv(AB0)*Td)

%Sistema de ecuaciones
L2A*cos(q2) - sin(q2)*(L3 + q3)==y*cos(q1) - L0*sin(q1) + z*sin(q1);
cos(q2)*(L3 + q3) + L2A*sin(q2)==L1 - x;
L2B==z*cos(q1) - L0*cos(q1) - y*sin(q1);


%-----------------------Soluciones de q1---------------------------------%

L2B==pz*cos(q1) - L0*cos(q1) - py*sin(q1)

%Pasamos de cartesianas a polares-->    A*sen(q1)+B*cos(q1)=C
A=-y;                                   %A=R*cos(alpha)
B=(z-L0);                               %B=R*sen(alpha)
C=L2B;
                                        %<=>R*cos(alpha)*sen(q1)+R*sen(alpha)*cos(q1)=C
R=sqrt(A.^2+B.^2);
alpha=atan2(B,A);
                            

%Seno de la suma-->R*sen(alpha+q1)=C
s1a=C./R;
c1a=sqrt(1-s1a.^2);

q1a=atan2(s1a,c1a)-alpha;
q1b=atan2(s1a,-c1a)-alpha;              
q1=[q1a,q1b]


%------------------------Soluciones q3------------------------------------% 

%Sistema de ecuaciones
cua1=simplify(der(1,4)^2+der(2,4)^2);
cua2=simplify(izq(1,4)^2+izq(2,4)^2); 
cua1==cua2;

2*L3*q3 + q3^2==(y*cos(q1)-L0*sin(q1)+z*sin(q1))^2+(L1 - x)^2 - L3^2 - L2A^2

%Ecuacion cuadratica-->a*q3^2 + b*q3 + c = 0
N=((y*cos(q1b) - L0*sin(q1b) + z*sin(q1b))^2 + (L1 - x)^2 -L3^2 -L2A^2);
a=1;
b=2*L3;
c=-N;

q3a=(-b-sqrt(b^2 - 4*a*c))/(2*a);
q3b=(-b+sqrt(b^2 - 4*a*c))/(2*a);      
q3=[q3a,q3b]


%--------------------Soluciones q2------------------------%

%A*sen(q1)+B*cos(q1)=C
A2=L2A;                                   %A=R*cos(alpha)
B2=(L3+q3);                               %B=R*sen(alpha)
C2=L1-x;
                                         %<=>R*cos(alpha)*sen(q1)+R*sen(alpha)*cos(q1)=C
R2=sqrt(A2.^2+B2.^2);
alpha2=atan2(B2,A2);


%Seno de la suma-->R*sen(alpha+q1)=C
s2a=C2./R2;
c2a=sqrt(1-s2a.^2);

q2a=atan2(s2a,c2a)-alpha2;
q2b=atan2(s2a,-c2a)-alpha2;              
q2=[q2a,q2b]



%% PARTE 5
clc
clear all
close all

syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms q1 q2 q3 q4 q5 q6 real 
syms phi theta psi real
syms nx ox ax x ny oy ay y nz oz az z
PI = sym(pi);
  
% theta_i d_i a_i alpha_i
% AB0=simplify(Aij(PI/2,L0,0,PI/2))
% A01=simplify(Aij(q1,L1,0,-PI/2))
% A12=simplify(Aij(q2,L2B,L2A,-PI/2))
% A23=simplify(Aij(0,L3+q3,0,0))
 
L0 = 1; 
L1=1.5;
L2A=0.5;
L2B=1.5;
L3=1;


%-----------------------Circunferencia------------------------------------%

%Centro en posicion HOME [0.5,0.5,2.5]
xc=1;
yc=0.5;
zc=3;

%Radio
R=1;   

%----------------------Loop para comprobar puntos-------------------------%

%Vaciamos 3 vectores para rellenarlo con los valores de las variables
%articulares de los 73 puntos
q1=zeros(1,73);
q2=zeros(1,73);
q3=zeros(1,73);
qaux=zeros(3,73);


theta=linspace(0,2*pi,73);
i=0;

for i= 1:73
    x(i)=xc+R*cos(theta(i));               %x=xc+r*cos(theta)
    y(i)=yc+R*sin(theta(i));               %y=yc+r*sen(theta)
    z(i)=zc;                                       %constante-->plano XY
    
    ver=CinematicaInversa([x(i),y(i),z(i)]);
    q1(i)=ver(1,2);
    q2(i)=ver(2,2);
    q3(i)=ver(3,2);
    
    
end


%--------------Trayectoria de las variables articulares-------------------%

figure(1);
ej1=subplot(3,1,1);
plot(theta,q1,'linewidth',1.5);
ylabel('q1');
title('Trayectoria circular plano XY')
grid;

ej2=subplot(3,1,2);
plot(theta,q2,'linewidth',1.5);
ylabel('q2');
grid;

ej3=subplot(3,1,3);
plot(theta,q3,'linewidth',1.5);
ylabel('q3');
grid;
xlabel('rad')


%-----------------Graficar la trayectoria en 3D---------------------------%

L0 = Link('revolute', 'd', L0, 'a', 0, 'alpha', pi/2, 'offset', pi/2);
L1 = Link('revolute', 'd', L1, 'a', 0, 'alpha', -pi/2, 'offset', 0);
L2 = Link('revolute', 'd', L2B, 'a', L2A, 'alpha', -pi/2, 'offset', 0);
L3 = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', 0, 'offset', 1);

L0.qlim = [-pi pi];
L1.qlim = [-pi pi];
L2.qlim = [-pi pi];
L3.qlim = [0 1.5];

robot = SerialLink([L0 L1 L2 L3], 'name', 'Robot');

figure(2);
qaux0 = [zeros(length(theta),1) q1',q2',q3'];
robot.plot(qaux0);
hold on;
figure(2)
hold on;
plot3(x, y, z, 'o-', 'LineWidth', 1.5, 'MarkerSize', 4);
grid on;
title('Trayectoria Circular');
axis equal; % Para mantener proporciones reales

while(1) 
    robot.plot(qaux0);
end

%% PARTE 6
clc
clear all
close all

syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms q1 q2 q3 q4 q5 q6 real 
syms phi theta psi real
syms nx ox ax x ny oy ay y nz oz az z
PI = sym(pi);
  

%-----------------------------Jacobiana-----------------------------------%
 
%Ecuaciones de posicion
x = L1 - cos(q2)*(L3 + q3) - L2A*sin(q2);                                
y = L2A*cos(q1)*cos(q2) - cos(q1)*sin(q2)*(L3 + q3) - L2B*sin(q1);       
z = L0 + L2B*cos(q1) - sin(q1)*sin(q2)*(L3 + q3) + L2A*cos(q2)*sin(q1);

%Jacobiano Directo
Jdir=[diff(x,q1),diff(x,q2),diff(x,q3);
      diff(y,q1),diff(y,q2),diff(y,q3);
      diff(z,q1),diff(z,q2),diff(z,q3)]

%Jacobiano Inverso
Jinv=inv(Jdir)


%Puntos singulares
DetJ=simplify(det(Jdir))
(L3 + q3)*(L3*sin(q2) - L2A*cos(q2) + q3*sin(q2))==0

%1)
L3+q3==0
% %-->q3=-L3 (no es un punto singular ya que q3=[0,1.5])

%2)
(L3*sin(q2) - L2A*cos(q2) + q3*sin(q2))==0
%sin(q2)*(L3+q3)= L2A*cos(q2)
%sin(q2)/cos(q2)=L2A/(L3+q3)

%La ecuación 
tan(q2)==L2A/(L3+q3) 

