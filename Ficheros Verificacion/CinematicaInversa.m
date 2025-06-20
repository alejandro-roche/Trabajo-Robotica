% -------------------------------------------------
% APARTADO A4 - Cinem�tica Inversa num�rica
% -------------------------------------------------
% LA FUNCI�N RECIBE DE ENTRADA UN VECTOR NUM�RICO CON LA POSICI�N EN 
% CARTESIANAS DEL EXTREMO DE LA ARTICULACI�N 3, (x,y,z)

% LA FUNCI�N DEBE DEVOLVER DOS VALORES:
% 1. UN VECTOR/MATRIZ NUM�RICO q CON LAS COORDENADAS ARTICULARES (q1,q2,q3)
%    SOLUCI�N. EN CASO DE M�LTIPLES SOLUCIONES, EL RESULTADO SER� UNA MATRIZ q,
%    DE MODO QUE CADA COLUMNA REPRESENTE UNA POSIBLE SOLUCI�N.
% 2. UNA VARIABLE L�GICA, fueraRango, QUE VALE VERDADERO (true) SI LA CINEM�TICA
%    INVERSA NO TIENE SOLUCI�N, EN CUYO CASO q SER� UNA VECTOR COLUMNA DE
%    VALOR 0, O FALSO (false) SI EXISTE SOLUCI�N.

function [q,fueraRango] = CinematicaInversa(xyz)
x = xyz(1);           % Posici�n cartesianas
y = xyz(2);           % 
z = xyz(3);           % 
fueraRango = false;   % Suponemos inicialmente que existe soluci�n


% SUSTITUYA los valores L0, L1, etc por sus valores num�ricos
L0 = 1; % Valor de ejemplo. Sustit�yalo por el que corresponda
  L1=1.5;
  L2A=0.5;
  L2B=1.5;
  L3=1;
  
% Escriba a continuaci�n las expresiones num�ricas de su cinem�tica
% inversa. Si hay varias soluciones posibles, cada columna de q ser� una 
% soluci�n separada (A continuaci�n tiene un ejemplo de la forma que
% tendr�a para el caso de dos soluciones

%------------------Soluciones de q1----------------------%

% Cambio de cartesianas a polares
%A*sen(q1)+B*cos(q1)=C
A=-y;                               %A=R*cos(alpha)                                                    
B=(z-L0);                           %B=R*sen(alpha)                  
C=L2B;                                         
R=sqrt(A.^2+B.^2);
alpha=atan2(B,A);

if abs(C) > abs(R)                  %Condicion para que el seno no sea >1               

    q = [0;0;0];
    return;
end

 %Seno de la suma-->                     
 s1a=C./R;
 c1a=sqrt(1-s1a.^2);

 q1a=atan2(s1a,c1a)-alpha;
 q1b=atan2(s1a,-c1a)-alpha;              
 q1=[q1a,q1b];                     %Soluciones de q1

%--------------Soluciones de q3--------------------------%

 N=(y.*cos(q1) - L0.*sin(q1) + z.*sin(q1)).^2 + (L1 - x)^2 -L3^2 -L2A^2;

 %a*q3^2 + b*q3 + c = 0
 a=1;
 b=2*L3;
 c=-N;

 q3a=(-b-sqrt(b^2 - 4*a*c))/(2*a);
 q3b=(-b+sqrt(b^2 - 4*a*c))/(2*a);
 

 q3=[q3a,q3b];

 %--------------Soluciones de q2--------------------------%

% Cambio de cartesianas a polares
%A*sen(q2)+B*cos(q2)=C
A2=L2A;                                   %A=R*cos(alpha)
B2=(L3+q3);                               %B=R*sen(alpha)
C2=L1-x;
                                        
R2=sqrt(A2.^2+B2.^2);
alpha2=atan2(B2,A2);

if abs(C2) > abs(R2)
    q = [0;0;0];
    return;
end

%Seno de la suma-->                     R*sen(alpha+q1)=C
s2a=C2./R2;
c2a=sqrt(1-s2a.^2);

q2a=atan2(s2a,c2a)-alpha2;
q2b=atan2(s2a,-c2a)-alpha2;             
q2=[q2a,q2b];

qaux = [q1, q1, q1 , q1; q2; q3, q3];
j = 0;

for i = 1:length(qaux)
     if(qaux(3,i)>-0.00001 && qaux(3,i)<0.00001 )
        qaux(3,i)=0.0000;

     end

     if  (qaux(3,i) >= 0 && qaux(3,i) <= 1.5)
        j = j+1;
        q(:,j) = qaux(:,i);

    end
end	
q(:, [1 2]) = q(:, [2 1]);

end