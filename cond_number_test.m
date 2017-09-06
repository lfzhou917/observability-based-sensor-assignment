A=[0, 0];
B=[2*sqrt(3),0];
C=[sqrt(3),0.1];
D = [sqrt(3),3];
O=[sqrt(3) 1];
AO =O-A;
BO =O-B;
CO=O-C;
DO=O-D;
axis equal; box on; hold on;
axis([-1 4 -1 4]);
plot (A(1), A(2), 'kd', 'MarkerSize',10), hold on
plot (B(1), B(2), 'kd', 'MarkerSize',10), hold on
plot (C(1), C(2), 'kd', 'MarkerSize',10), hold on
plot (D(1), D(2), 'kd', 'MarkerSize',10), hold on
plot (O(1), O(2), 'rp', 'MarkerSize',10), hold on
invconda=1/cond([AO;BO]);
%sa = svds([AO;BO],2);
invcondb=1/cond([AO;CO]);
%sb = svds([AO;CO],2);
invcondc=1/cond([BO;CO]);
%sc = svds([BO;CO],2);
invcondd=1/cond([AO;BO;CO]);
invconde=1/cond([AO;BO;CO;DO]);
%sd = svds([AO;BO;CO],2);
%error=invcondd-max([invconda invcondb invcondc]);
%%
AO = [1 0];
BO =[-1 0];
CO=[0 1];
D=[0 -1];
invcond1=1/cond([AO;BO]);
invcond2=1/cond([AO;CO]);
invcond3=1/cond([AO;D]);
invcond4=1/cond([BO;CO]);
invcond5=1/cond([BO;D]);
invcond6=1/cond([CO;D]);
invcond7=1/cond([AO;BO;CO;D]);

error=invcond7-max([invcond1 invcond2 invcond3 invcond4 invcond5 invcond6]);
%%
x = 0:0.1:100;
y = exp(-100./x);
plot(x,y)