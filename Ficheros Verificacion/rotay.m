function R=rotay(beta)
    R=[cos(beta) 0 sin(beta) 0;
       0 1 0 0;
       -sin(beta) 0 cos(beta) 0;
       0 0 0 1];