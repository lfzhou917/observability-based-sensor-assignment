% use pair sensors p1 and p2 to obtain the invercond of the p1-p2-t
% t indicates target
function [pair_invercond] = pair_invercond(p1,p2,t)
% calculate distance dis(p1,t) and dis(p2,t)
d_p1_t=norm(p1-t);
d_p2_t=norm(p2-t);
% distance ratio
r=d_p1_t/d_p2_t;
% calculate the angle between the vector p1_t and p2_t
theta=atan2(norm(cross([p1(1)-t(1),p1(2)-t(2),0],...
                     [p2(1)-t(1),p2(2)-t(2),0])),dot([p1(1)-t(1),p1(2)-t(2),0],...
                     [p2(1)-t(1),p2(2)-t(2),0]));

% maximum and minimum eigenvalues of observability matrix. 
lambda_min=(1+r^2-sqrt(1+r^4+2*r^2*cos(2*theta)))/2;
lambda_max=(1+r^2+sqrt(1+r^4+2*r^2*cos(2*theta)))/2;
% calculate the inverse of the condition number for p1_p2_t system          
pair_invercond=sqrt(lambda_min/lambda_max);
end