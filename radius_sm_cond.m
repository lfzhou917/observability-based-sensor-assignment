function [sm_cond_r]=radius_sm_cond(s1,s2,s3,cov_r)
% initialize the sensors
%s1=[0,1];
%s2=[-sqrt(3)/2, -1/2];
%s3=[sqrt(3)/2, -1/2];
% draw the disk
%cov_r = (1+sqrt(3))/2; % the radius of covering circle
%cov_r=2;
r = 0:0.05:cov_r;
th = -pi:pi/50:pi;
[R, Th] = meshgrid(r, th);
[X,Y] = pol2cart(Th,R);

[Nraw, Ncol]=size(X);
Invcond=zeros(Nraw, Ncol);

for i=1: Nraw
    for j=1:Ncol
        s12_incond=pair_invercond(s1,s2,[X(i,j),Y(i,j)]);
        s13_incond=pair_invercond(s1,s3,[X(i,j),Y(i,j)]);
        s23_incond=pair_invercond(s2,s3,[X(i,j),Y(i,j)]);
        if s12_incond>=s13_incond && s12_incond>=s23_incond
            %plot(X(i,j),Y(i,j),'r*'); hold on
            Invcond(i,j)=s12_incond;
        elseif s13_incond>=s12_incond && s13_incond>=s23_incond
            %plot(X(i,j),Y(i,j),'b+'); hold on
            Invcond(i,j)=s13_incond;
        elseif s23_incond>=s13_incond && s23_incond>=s12_incond
            %plot(X(i,j),Y(i,j),'gx'); hold on
            Invcond(i,j)=s23_incond;
        else
        end
    end
end
 sm_cond_r=min(Invcond(:));
end
