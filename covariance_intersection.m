% 
C1=[5 1; 1 2];
C2=[2 -1; -1 6];
[V1,E1] = eig(C1);
%V1*sqrtm(E1)*V1'*V1*sqrt(E1)*V1'
T1=inv(V1*sqrtm(E1));
C11=T1*C1*T1'; % keep in mind, only det(C1)>0, C11=I
C21=T1*C2*T1';
[V21,E21] = eig(C21);
T=V21*T1;
% V21*C21*V21'
% T*C2*T'
% V21*T1*C2*T1'*V21'
C111=T*C1*T';
C211=T*C2*T';
inv(T)
V1*sqrtm(E1)*V21
