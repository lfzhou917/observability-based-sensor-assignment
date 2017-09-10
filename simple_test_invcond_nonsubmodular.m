% 
N=3;
M=1;
pr=[0, 0; 2*sqrt(3), -9; sqrt(3),3];
pt=[sqrt(3),1];
uo_max=1;
ob_weight=invercond(pr,pt,uo_max,N,M);

