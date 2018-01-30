syms x y z
obs = [x^2, x*y, x*z; y*x, y^2, y*z; z*x, z*y, z^2];
eig_obs=eig(obs);


%%
syms di dj dk phi_i phi_j phi_k theta_i theta_j theta_k 
assume(di>=0)
assume(dj>=0)
assume(dk>=0)

assume(phi_i>=0 & phi_i<=pi)
assume(phi_j>=0 & phi_j<=pi)
assume(phi_k>=0 & phi_k<=pi)

assume(theta_i>=0 & theta_i<=2*pi)
assume(theta_j>=0 & theta_j<=2*pi)
assume(theta_k>=0 & theta_k<=2*pi)
assumptions

%calculate three vectors
vi=[di*sin(phi_i)*cos(theta_i), di*sin(phi_i)*sin(theta_i), di*cos(phi_i)];
vj=[dj*sin(phi_j)*cos(theta_j), dj*sin(phi_j)*sin(theta_j), dj*cos(phi_j)];
vk=[dk*sin(phi_k)*cos(theta_k), dk*sin(phi_k)*sin(theta_k), dk*cos(phi_k)];

%calcuate the corss angle between
angle_ij = atan2(norm(cross(vi,vj)), dot(vi,vj)); 
angle_ik = atan2(norm(cross(vi,vk)), dot(vi,vk)); 
angle_jk = atan2(norm(cross(vj,vk)), dot(vj,vk)); 


%obs and its eigen
obs=[vi;vj;vk];
obs_oug=obs'*obs;
eig_obs_oug=eig(obs_oug);

%mineig_obs_oug=min(eig_obs_oug);
%maxeig_obs_oug=max(eig_obs_oug);

%cond_obs = sqrt(eig_obs_oug/maxeig_obs_oug);
[abbr_eig_obs_oug, angle_ij]...
    = subexpr(eig_obs_oug, 'angle_ij');
%%
syms  oj_x oj_y oj_z ok_x ok_y ok_z x
%syms oi_x oi_y  oi_z
oi_x=1; oi_y=0;  oi_z=0;

obs_cat=[oi_x oi_y oi_z; oj_x oj_y oj_z; ok_x ok_y ok_z];

eig_obs_cat=eig(obs_cat);

s = svd(obs_cat);
% eqn = - x^3 + x^2*(oi_x+oi_y+oi_z) ...
%     -x*(oi_x*oj_y+oi_x*ok_z+oj_y*ok_z - oi_y*oj_x- oi_z*ok_x-oj_z*ok_y)...
%     +oi_x*(oj_y*ok_z-oj_z*ok_y) - oi_y*(oj_x*ok_z-oj_z*ok_x) + oi_z*(oj_x*ok_y-ok_x*oj_y)==0; 
% 
% solx = solve(eqn,x); 

%cond_obs_cat = cond(obs_cat);


