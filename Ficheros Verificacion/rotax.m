function R=rotax(alpha)
    R=[1 0 0 0;
       0 cos(alpha) -sin(alpha) 0;
       0 sin(alpha) cos(alpha) 0;
       0 0 0 1];