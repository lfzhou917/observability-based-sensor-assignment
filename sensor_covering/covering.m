clc;
% initialize the sensors
s1=[0,1];
s2=[-sqrt(3)/2, -1/2];
s3=[sqrt(3)/2, -1/2];
% draw the disk
%cov_r = (1+sqrt(3))/2; % the radius of covering circle
cov_r=1;
r = 0:0.05:cov_r;
th = -pi:pi/50:pi;
% dx=0;
% dy=0;
% for r=0:0.1:cov_r
%     for th = 0:0.1:2*pi
%           dx=[dx; r*cos(th)];
%           dy=[dy; r*sin(th)];
%     end
% end     
% N=length(dx);
[R, Th] = meshgrid(r, th);
[X,Y] = pol2cart(Th,R);

[Nraw, Ncol]=size(X);
Invcond=zeros(Nraw, Ncol);
%Sm_invcond=zeros(Nraw, Ncol);

figure(1)
axis equal, hold on
%h1 = plot(xunit, yunit);
h2 = plot(s1(1), s1(2), 'ko');
h3 = plot(s2(1), s2(2), 'ko');
h4 = plot(s3(1), s3(2), 'ko'); 
% select the best pair (s1, s2)_red*(r*), (s1_s3)_blue+(b+), (s2,
% s3)_greenx(gx)

for i=1: Nraw
    for j=1:Ncol
        s12_incond=pair_invercond(s1,s2,[X(i,j),Y(i,j)]);
        s13_incond=pair_invercond(s1,s3,[X(i,j),Y(i,j)]);
        s23_incond=pair_invercond(s2,s3,[X(i,j),Y(i,j)]);
        Sm_invcond(i,j)=min([s12_incond,s13_incond,s23_incond]);
%     s12_incond=pair_invercond(s1,s2,[dx(i),dy(i)]);
%     s13_incond=pair_invercond(s1,s3,[dx(i),dy(i)]);
%     s23_incond=pair_invercond(s2,s3,[dx(i),dy(i)]);
        if s12_incond>=s13_incond && s12_incond>=s23_incond
            plot(X(i,j),Y(i,j),'r*'); hold on
             %plot3(dx(i),dy(i),s12_incond); hold on
             Invcond(i,j)=s12_incond;
        elseif s13_incond>=s12_incond && s13_incond>=s23_incond
            plot(X(i,j),Y(i,j),'b+'); hold on
            %plot3(dx(i),dy(i),s13_incond); hold on
            Invcond(i,j)=s13_incond;
        elseif s23_incond>=s13_incond && s23_incond>=s12_incond
            plot(X(i,j),Y(i,j),'gx'); hold on
            %plot3(dx(i),dy(i),s23_incond); hold on
            Invcond(i,j)=s23_incond;
        else
        end
    end
end
 min(Invcond(:))
figure(2)
% axis equal, hold on
% %h1 = plot(xunit, yunit);
% h2 = plot(s1(1), s1(2), 'ko');
% h3 = plot(s2(1), s2(2), 'ko');
% h4 = plot(s3(1), s3(2), 'ko'); 
surf(X, Y, Invcond);

 %figure(3)
 %surf(X, Y, Sm_invcond);

%%%
% cx=0;
% cy=0;
% cov_r=2;
% for r=0:0.05:cov_r
%     for th = 0:pi/50:2*pi
%           x=r*cos(th) + cx;
%           y=r*sin(th) + cy;
%           s12_incond=pair_invercond(s1,s2,[x,y]);
%           s13_incond=pair_invercond(s1,s3,[x,y]);
%           s23_incond=pair_invercond(s2,s3,[x,y]);
%           sm_inv_temp=[sm_inv_temp, min([s12_incond,s13_incond,s23_incond])];       
%     end
%     
% end     
% N=length(x);