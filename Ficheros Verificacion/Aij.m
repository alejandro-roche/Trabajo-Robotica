function A=Aij(theta,d,a,alpha)

    A=rotaz(theta)*T(0,0,d)*T(a,0,0)*rotax(alpha);