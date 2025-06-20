% -------------------------------------------------
% APARTADO A3 - Cinem�tica directa num�rica 
% -------------------------------------------------

% IMPORTANTE: ESTA FUNCI�N DEBE DEVOLVER DOS VECTORES NUM�RICOS, POSICI�N Y ORIENTACI�N,
% CUANDO RECIBE DE ENTRADA UN VECTOR TAMBI�N NUM�RICO CON LAS TRES COORDENADAS ARTICULARES
% q1, q2 y q3

function [xyz,angEuler] = CinematicaDirecta(in)
  q1       = in(1);           % Posiciones articulares
  q2       = in(2);            
  q3       = in(3);

  % ------ SOLO DEBE ALTERAR LAS EXPRESIONES ENTRE ESTAS L�NEAS ----------
  % SUSTITUYA los valores L0, L1, etc por sus valores num�ricos  
  L0 = 1; % Valor de ejemplo. Sustit�yalo por el que corresponda
  L1=1.5;
  L2A=0.5;
  L2B=1.5;
  L3=1;
  
  % Posici�n del extremo de la articulaci�n 3
  
  x = L1 - cos(q2)*(L3 + q3) - L2A*sin(q2);                                % Ecuaci�n de ejemplo. C�mbiela por su soluci�n
  y = L2A*cos(q1)*cos(q2) - cos(q1)*sin(q2)*(L3 + q3) - L2B*sin(q1);       % Ecuaci�n de ejemplo. C�mbiela por su soluci�n
  z = L0 + L2B*cos(q1) - sin(q1)*sin(q2)*(L3 + q3) + L2A*cos(q2)*sin(q1);  % Ecuaci�n de ejemplo. C�mbiela por su soluci�n
  
  % Orientaci�n (�ngulos de Euler) del marco de referencia 3
  phi=atan2(-cos(q2),cos(q1)*sin(q2));
  psi=atan2(cos(q2)*sin(q1),-cos(q1));
  theta=atan2(cos(q2),-sin(psi)*sin(q2));
  % ----------------------------------------------------------------------  
  % Variables de salida. No las modifique.
  xyz=[x;y;z];
  angEuler = [phi;theta;psi]; % �ngulos de Euler seg�n convenio ZXZ
end


  