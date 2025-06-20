% -------------------------------------------------
% APARTADO A4 - Cinemática Inversa simbólica
% -------------------------------------------------

function q = CinematicaInversaSimbolica
  
  % Emplee exclusivamente las siguientes variables simbólicas en sus expresiones
  syms x y z q1 q2 q3  real % Coordenadas cartesianas del efector final
  syms L0 L1 L1A L1B L2 L2A L2B L3 L3A L3B real % Parámetros dimensionales
  % Emplee esta expresión de pi simbólico
  PI = sym(pi);

  % ------ SOLO DEBE ALTERAR LAS EXPRESIONES ENTRE ESTAS LÍNEAS ----------
  % Escriba a continuación las expresiones simbólicas de su cinemática
  % inversa. 
  A=-y;                               %A=R*cos(alpha)                                                    
B=(z-L0);                           %B=R*sen(alpha)                  
C=L2B;                                         
R=sqrt(A.^2+B.^2);
alpha=atan2(B,A);

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


%Seno de la suma-->                     R*sen(alpha+q1)=C
s2a=C2./R2;
c2a=sqrt(1-s2a.^2);

q2a=atan2(s2a,c2a)-alpha2;
q2b=atan2(s2a,-c2a)-alpha2;             
q2=[q2a,q2b];
  % ----------------------------------------------------------------------  
  % Variables de salida. No las modifique.                                           
  q=[q1, q1, q1 , q1; q2; q3, q3];
end



  