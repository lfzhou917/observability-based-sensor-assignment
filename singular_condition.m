% singualr value and conditon plot
clc;
theta = -pi:0.1:pi;
r=0:0.1:5;
uo_max=1;

%surf(invcond);
[R, Theta] = meshgrid(r, theta);
temp=ones(size(R));
%di_temp=ones(size(R));
di=1;
uo_max_temp=ones(size(R));

lambda_max=(temp + R.^2 + sqrt(temp + R.^4 + 2 * R.^2  .* cos(2 * Theta)))/2;
lambda_min=(temp + R.^2 - sqrt(temp + R.^4 + 2 * R.^2  .* cos(2 * Theta)))/2;

singular_max=sqrt(temp + R.^2 + sqrt(temp + R.^4 + 2 * R.^2  .* cos(2 * Theta)))/2;
singular_min=sqrt(temp + R.^2 - sqrt(temp + R.^4 + 2 * R.^2  .* cos(2 * Theta)))/2;

 Invcond_o = singular_min./singular_max;
 Invcond_ou=sqrt(lambda_min./(lambda_max+uo_max_temp.^2/di)); 

figure (1)
title('inverese of condition_3d')
%mesh(R, Theta, real(Invcond_ou))
surf(R, Theta, real(Invcond_ou))
figure (2)
title('inverse of condition contour')
contour(R,Theta, real (Invcond_ou))

figure (3)
 [X,Y] = pol2cart(Theta,R);
surf(X, Y, real(Invcond_ou))
figure (4)
H=polar(X,Y);
hold on;
contour(X, Y, real(Invcond_ou));
%set(h,'Visible','off')
%axis off
%axis image

% 
% % figure (1)
% % title('inverese of condition_3d')
% % mesh(R, Theta, real(Invcond_ou))
% % figure (2)
% % title('inverse of condition contour')
% % contour(R,Theta, real (Invcond_ou))
% 
% figure (3)
% title('singular_min_3d')
% mesh(R, Theta, real(Invcond_o))
% figure (4)
% title('singular_min_contour')
% contour(R,Theta, real (Invcond_o))


%  
% figure (5)
% title('singular_max_3d')
% mesh(R, Theta, real(singular_max))
% figure (6)
% title('singular_max_contour')
% contour(R,Theta, real (singular_max))