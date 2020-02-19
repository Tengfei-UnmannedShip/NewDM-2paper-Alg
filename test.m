A=[1 Inf 3 4
    5 6 7 Inf
    9 Inf 11 12];


I = find(A==Inf);
J = find(A~=Inf);
A1 = A; 
A1(I) = mmax(A(J));

