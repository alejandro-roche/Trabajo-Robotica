

function [Tau_add] = Compensacion(in)
% Variables de entrada en la funcion: [q(1)  q(2)  q(3)]

q1        = in(1);
q2        = in(2);
q3        = in(3);



g=9.8;

% Par gravitatorio                
Ga=[-1.0*g*(172.19*sin(q1) - 90.195*cos(q1)*cos(q2) + 32.798*cos(q1)*sin(q2) + 65.596*q3*cos(q1)*sin(q2));
    -8.1996*g*sin(q1)*(4.0*cos(q2) + 11.0*sin(q2) + 8.0*q3*cos(q2));
     -65.596*g*sin(q1)*sin(q2)];
       

  Tau_add=Ga;
