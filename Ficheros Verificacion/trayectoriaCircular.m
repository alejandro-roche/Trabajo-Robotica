% -------------------------------------------------
% APARTADO A5 - Trayectoria circular en cartesianas
% -------------------------------------------------

function [x,y,z,q1,q2,q3] = trayectoriaCircular
  
  % Desarrolle aquí el código necesario para calcular una trayectoria 
  % circular en cartesianas, cuyos valores deberá guardar en los vectores x,y,z
  % así como las correspondiente coordenadas articulares q1,q2,q3.
  % El código será tal que para cualquier componente i de los vectores
  
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
    x(i)=xc+R*cos(theta(i));                       %x=xc+r*cos(theta)
    y(i)=yc+R*sin(theta(i));                       %y=yc+r*sen(theta)
    z(i)=zc;                                       %constante-->plano XY
    
    ver=CinematicaInversa([x(i),y(i),z(i)]);
    q1(i)=ver(1,2);
    q2(i)=ver(2,2);
    q3(i)=ver(3,2);
    
    
end

  end



  