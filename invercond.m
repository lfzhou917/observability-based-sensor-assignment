function [ob_weight] = invercond(pr,pt,uo_max,N,M)
%here is the sensor view, since it doesn't know the control input of
%target, it can only calculate the lower bound of the inverse of cond
%bound. it assumes the target has the maximum motion ability u_max=1;

% N=3; % the number of robot
% M=2; % the number of target
% pr=[0 0; 4 0; 4 4];
% pt=[1 2; 3 1];
% uo_max=1;

d_ri_t=zeros(N,M); % the distance between  robot i and target
d_rj_t=zeros(N,M); % the distance between  robot j and target

% graph weight defined as the lower bound of inverese of the ...
% condition number between each pair of robots and the target
ob_weight=zeros(N*N,M);

% the ratio of  distance between two robots, w.r.t. target
rij=zeros(N*N,M);

% the angle between  two robots, w.r.t. target 
thetaji=zeros(N*N,M);

% maximum and minimum eigenvalue
lambda_max=zeros(N*N,M);
lambda_min=zeros(N*N,M);


for i=1:N
    for j=1:N 
             for k=1:M
                 d_ri_t(i,k)=sqrt((pr(i,1)-pt(k,1))^2+(pr(i,2)-pt(k,2))^2);   
                 d_rj_t(j,k)=sqrt((pr(j,1)-pt(k,1))^2+(pr(j,2)-pt(k,2))^2);
                 % the ratio of two robots' distances, di and dj, dj/di, w.r.t. target k
                 rij(N*(i-1)+j, k)=d_rj_t(j,k)/d_ri_t(i,k);
                 % the angle between two robots, i, j, w.r.t. target k
                 thetaji(N*(i-1)+j, k)=atan2(norm(cross([(pr(i,1)-pt(k,1)),(pr(i,2)-pt(k,2)),0],...
                     [(pr(j,1)-pt(k,1)),(pr(j,2)-pt(k,2)),0])),dot([(pr(i,1)-pt(k,1)),(pr(i,2)-pt(k,2)),0],...
                     [(pr(j,1)-pt(k,1)),(pr(j,2)-pt(k,2)),0]));
                 
                 lambda_max(N*(i-1)+j, k)=(1 + rij(N*(i-1)+j, k)^2 +...
                     sqrt(1 + rij(N*(i-1)+j, k)^4 + 2 * rij(N*(i-1)+j, k)^2  * cos(2 * thetaji(N*(i-1)+j, k))))/2;
                 lambda_min(N*(i-1)+j, k)=(1 + rij(N*(i-1)+j, k)^2 -...
                     sqrt(1 + rij(N*(i-1)+j, k)^4 + 2 * rij(N*(i-1)+j, k)^2  * cos(2 * thetaji(N*(i-1)+j, k))))/2;
                 %robot pair i, j and target k
                 ob_weight(N*(i-1)+j,k)=sqrt(lambda_min(N*(i-1)+j, k)/(lambda_max(N*(i-1)+j, k)+...
                     uo_max^2/d_ri_t(i,k)^2));
             end     
    end
end

end