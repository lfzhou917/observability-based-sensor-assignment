function  [tarx_true, tary_true, ux_op, uy_op, invercond_max]=target_escape_game(p1x,p1y,p2x,p2y,tarx_true, tary_true)
global u_max
%global tarx_true tary_true
% the target will select a control in the  disc set 
% to minimize the observability metric w.r.t. true target

% u_theta = 0:0.1:2*pi;
% u_r=0:0.1:u_max;

% [u_R, u_Theta] = meshgrid(u_r, u_theta);
% 
% u_x=u_R.*cos(u_Theta);
% u_y=u_R.*sin(u_Theta);
ux=0;
uy=0;
for u_r=0:0.1:u_max
    for u_theta = 0:0.1:2*pi
          ux=[ux; u_r*cos(u_theta)];
          uy=[uy; u_r*sin(u_theta)];
    end
end

% compare to select the best to minizine the observability
%  define the ux_op, uy_op

invercond_max=1;

sizeux=694;

tarx_next=zeros(sizeux,1);
tary_next=zeros(sizeux,1);
d_1_o=zeros(sizeux,1);
d_2_o=zeros(sizeux,1);
r_12_o=zeros(sizeux,1);
theta_12_o=zeros(sizeux,1);
lambda_min_12=zeros(sizeux,1);
lambda_max_12=zeros(sizeux,1);
ob_cond=zeros(sizeux,1);
invercond=zeros(sizeux,1);

           
%  target knows the relative positon and control input, thus, it can
%  calculate the inverse of the cond of the ob directly
% ob_matrix=[p1x-tarx_next, p1y-tary_next;...
%     p2x-tarx_next, p2y-tary_next...
%     ux, uy];
 
    for i=1: size(ux) 
        % 
        tarx_next(i)=tarx_true+ux(i);
        tary_next(i)=tary_true+uy(i);
        
%         % the distance between best pair and target_next
%         d_1_o(i)=sqrt((p1x-tarx_next(i))^2+(p1y-tary_next(i))^2);
%         d_2_o(i)=sqrt((p2x-tarx_next(i))^2+(p2y-tary_next(i))^2);
% 
%         r_12_o(i)=d_2_o(i)/d_1_o(i);
% 
%         % the theta between two vectors
%         theta_12_o(i)=atan2(norm(cross([(p1x-tarx_next(i)),(p1y-tary_next(i)),0],...
%                              [(p2x-tarx_next(i)),(p2y-tary_next(i)),0])),dot([(p1x-tarx_next(i)),(p1y-tary_next(i)),0],...
%                              [(p2x-tarx_next(i)),(p2y-tary_next(i)),0]));
% 
% 
%         % %maximum and minimum eigenvalues of observability matrix. 
%         lambda_min_12(i)=(1+r_12_o(i)^2-sqrt(1+r_12_o(i)^4+2*r_12_o(i)^2*cos(2*theta_12_o(i))))/2;
%         lambda_max_12(i)=(1+r_12_o(i)^2+sqrt(1+r_12_o(i)^4+2*r_12_o(i)^2*cos(2*theta_12_o(i))))/2; 
% 
%         invercond(i)=sqrt(lambda_min_12(i)/(lambda_max_12(i)+u_max^2/d_1_o(i)^2));        
            
            % calculate the inverse of the condition number directly
                ob_cond(i)=cond([tarx_next(i)-p1x, tary_next(i)-p1y;...
                                      tarx_next(i)-p2x, tary_next(i)-p2y;...
                                      ux(i), uy(i)]);
                                  
                 invercond(i)=1/ob_cond(i);
            
             if  invercond(i)<invercond_max
                  invercond_max=invercond(i);
                  ux_op=ux(i);
                  uy_op=uy(i);
             else

             end
         
    end
           % uodate the true position of the target
           tarx_true=tarx_true+ux_op;
           tary_true=tary_true+uy_op;
end