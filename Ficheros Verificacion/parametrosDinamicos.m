% -------------------------------------------------
% APARTADO B1 - Parámetros dinámicos
% -------------------------------------------------

function [s11,s22,s33,I11,I22,I33] = parametrosDinamicos
% DATOS DINÁMICOS DEL BRAZO DEL ROBOT
% Consigne datos numéricos, no simbólicos
% Eslabón 1
  s11 = [ 0, 0.7500, 0]'; % m
  I11=[18.4490 , 0 , 0 ; 
      0, 0.8068 , 0;
      0 ,0 ,18.4490 ]; % kg.m2

 

% Eslabón 2
  s22 = [-0.0625 , 0.9375 , 0]'; % m
  I22=[56.0744 , 4.6123 , 0 ; 
      4.6123, 27.9139 ,0;
      0,0 , 59.3928]; % kg.m2 

% Eslabón 3  
  s33 = [ 0,  0,  -0.5000]'; % m
  I33=[5.4664 , 0 , 0 ; 
      0, 5.4664  ,0;
      0,0 , 0.5379]; % kg.m2 
end