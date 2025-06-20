% -------------------------------------------------
% APARTADO B1 - Modelo dinámico inverso
% -------------------------------------------------

function [qdd] = ModeloDinamico_R3GDL(in)
% Variables de entrada en la funcion: [q(3)  qp(3)  Tau(3)]
q1        = in(1);
q2        = in(2);
q3        = in(3);
qd1       = in(4);
qd2       = in(5);
qd3       = in(6);
Tau1      = in(7);
Tau2      = in(8);
Tau3      = in(9);

% Matriz de Inercias
g=9.8;
Ma=[32.798*q3 + 11.751*cos(2.0*q2) - 16.399*sin(2.0*q2) - 32.798*q3*cos(2.0*q2) - 32.798*q3*sin(2.0*q2) - 32.798*q3^2*cos(2.0*q2) + 32.798*q3^2 + 257.66, 49.197*cos(q2) + 86.095*sin(q2) + 98.395*q3*cos(q2), 98.395*sin(q2);
                                                                                                          86.095*sin(q2) + 98.395*cos(q2)*(q3 + 0.5),                    65.596*q3^2 + 65.596*q3 + 69.625,         32.798;
                                                                                                                                      98.395*sin(q2),                                              32.798,         69.863];
 
 
% Matriz de aceleraciones centrípetas y de Coriolis
Va=[0.040639*qd1 + 86.095*qd2^2*cos(q2) - 49.197*qd2^2*sin(q2) + 32.798*qd1*qd3 + 196.79*qd2*qd3*cos(q2) - 98.395*q3*qd2^2*sin(q2) + 65.596*q3*qd1*qd3 - 32.798*qd1*qd2*cos(2.0*q2) - 32.798*qd1*qd3*cos(2.0*q2) - 23.501*qd1*qd2*sin(2.0*q2) - 32.798*qd1*qd3*sin(2.0*q2) - 65.596*q3*qd1*qd2*cos(2.0*q2) - 65.596*q3*qd1*qd3*cos(2.0*q2) + 65.596*q3*qd1*qd2*sin(2.0*q2) + 65.596*q3^2*qd1*qd2*sin(2.0*q2);
    0.1412*qd2 + 65.596*qd2*qd3 + 16.399*qd1^2*cos(2.0*q2) + 11.751*qd1^2*sin(2.0*q2) + 32.798*q3*qd1^2*cos(2.0*q2) - 32.798*q3*qd1^2*sin(2.0*q2) - 32.798*q3^2*qd1^2*sin(2.0*q2) + 131.19*q3*qd2*qd3;
    0.18402*qd3 - 32.798*qd1^2*sin(q2)^2 - 65.596*q3*qd2^2 + 16.399*qd1^2*sin(2.0*q2) - 32.798*qd2^2 - 65.596*q3*qd1^2*sin(q2)^2];
 
% Par gravitatorio                
Ga=[ -1.0*g*(172.19*sin(q1) - 15.0*cos(q2) + 30.0*sin(q2) - 90.195*cos(q1)*cos(q2) + 32.798*cos(q1)*sin(q2) + 30.0*q3*sin(q2) + 65.596*q3*cos(q1)*sin(q2));
     -8.1996*g*sin(q1)*(4.0*cos(q2) + 11.0*sin(q2) + 8.0*q3*cos(q2));
     -65.596*g*sin(q1)*sin(q2)];
 
% Ecuación del robot
%    Tau = M*qpp + V + G
  Tau=[Tau1;Tau2;Tau3];

% Por lo que:  
% Aceleraciones
  qdd = inv(Ma)*(Tau-Va-Ga);
  