% -------------------------------------------------
% APARTADO A3 - Cinemática directa simbólica 
% -------------------------------------------------

function [xyz] = CinematicaDirectaSimbolica
  
  % Emplee exclusivamente las siguientes variables simbólicas en sus expresiones
  syms x y z q1 q2 q3  real % variables articulares 
  syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B real % Parámetros dimensionales
  PI = sym(pi);   % Emplee esta expresión de pi simbólico

  % ------ SOLO DEBE ALTERAR LAS EXPRESIONES ENTRE ESTAS LÍNEAS ----------
  % NO SUSTITUYA los valores L0, L1, etc por sus valores numéricos
  
  % Posición del extremo de la articulación 3
  x = L1 - cos(q2)*(L3 + q3) - L2A*sin(q2);                                   % Ecuación de ejemplo. Cámbiela por su solución
  y = L2A*cos(q1)*cos(q2) - cos(q1)*sin(q2)*(L3 + q3) - L2B*sin(q1);          % Ecuación de ejemplo. Cámbiela por su solución
  z = L0 + L2B*cos(q1) - sin(q1)*sin(q2)*(L3 + q3) + L2A*cos(q2)*sin(q1);     % Ecuación de ejemplo. Cámbiela por su solución
  
  % ----------------------------------------------------------------------  
  % Variables de salida. No las modifique.
  xyz=[x;y;z];
  end



  