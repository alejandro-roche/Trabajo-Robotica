% -------------------------------------------------
% APARTADO A6 - Jacobianos
% -------------------------------------------------

function [Jdir,Jinv] = jacobiano
  % Desarrolle aquí el código necesario para determinar los Jacobianos 
  % directo (Jdir) e inverso (Jinv)
  % El cálculo se realizará de modo exclusivamente simbólico

% Definición de variables simbólicas  
syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B L4 L4A L4B L5 L5A L5B L6 L6A L6B real  
syms x y z q1 q2 q3 real  
PI = sym(pi); % use PI (en mayúsculas) para referirse a pi.
 
% Jacobiano directo  
Jdir = zeros(3,3); % Cámbielo por su solución
% Jacobiano inverso  
Jinv = zeros(3,3); % Cámbielo por su solución
  
%Ecuaciones de posicion
x = L1 - cos(q2)*(L3 + q3) - L2A*sin(q2);                                
y = L2A*cos(q1)*cos(q2) - cos(q1)*sin(q2)*(L3 + q3) - L2B*sin(q1);       
z = L0 + L2B*cos(q1) - sin(q1)*sin(q2)*(L3 + q3) + L2A*cos(q2)*sin(q1);

%Jacobiano Directo
Jdir=[diff(x,q1),diff(x,q2),diff(x,q3);
      diff(y,q1),diff(y,q2),diff(y,q3);
      diff(z,q1),diff(z,q2),diff(z,q3)];

%Jacobiano Inverso
Jinv=inv(Jdir);

end



  