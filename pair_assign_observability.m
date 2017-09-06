function [pair_id,low_inv_cond_id]=pair_assign_observability(N,id,px,py,tarx_hat,tary_hat,Sigma_hat)%Sigma_hat

% id indicates the label of the robot, that needs a best pair to pair with
% initially, set pair j the same as i

% calculate the distance between every robot and the target
%d_i_o=zeros(N,1);

% %the Mahalanobis distance
 md_i_o=zeros(N,1);
% the ratio of d_i_o/d_id_o
r_ido=zeros(N,1);
% the angle between  id and i, w.r.t. target o 
theta_id_i=zeros(N,1);

% the maximum bound of the control input 
global u_max


% the lower bound for itself is 0;
low_inv_cond_id=0;

% the euclidean distance between the robot id and the targt 
%d_id_o=sqrt((px(id)-tarx_hat)^2+(py(id)-tary_hat)^2);

% % the Mahalanobis distance
 md_id_o=sqrt([(px(id)-tarx_hat);(py(id)-tary_hat)]'*inv(Sigma_hat)*[(px(id)-tarx_hat);(py(id)-tary_hat)]);


% maximum and minimum eigenvalue
lambda_max_idi=zeros(N,1);
lambda_min_idi=zeros(N,1);

% the lower bound inverse of the cond
low_inv_cond=zeros(N,1);

% calculate the lower bound of the inverse of cond
% compare to select the maximum
    for i=1:N
           %* calculate euclidean distance between every robot and the target
            % d_i_o(i)=sqrt((px(i,1)-tarx_hat)^2+(py(i,1)-tary_hat)^2);  
%            % the Mahalanobis distance
           md_i_o(i)=sqrt([(px(i)-tarx_hat);(py(i)-tary_hat)]'*...
                inv(Sigma_hat)*[(px(i)-tarx_hat);(py(i)-tary_hat)]);
%          % the ratio of d_i_o/d_id_o or 
             r_ido(i)=md_i_o(i)/md_id_o;
   %             r_ido(i)=d_i_o(i)/d_id_o;
         % the angle between  id and i, w.r.t. target o 
             theta_id_i(i)=atan2(norm(cross([(px(id)-tarx_hat),(py(id)-tary_hat),0],...
                     [(px(i)-tarx_hat),(py(i)-tary_hat),0])),dot([(px(id)-tarx_hat),(py(id)-tary_hat),0],...
                     [(px(i)-tarx_hat),(py(i)-tary_hat),0]));
                 
         %maximum and minimum eigenvalues of observability matrix. 
             lambda_min_idi(i)=(1+r_ido(i)^2-sqrt(1+r_ido(i)^4+2*r_ido(i)^2*cos(2*theta_id_i(i))))/2;
             lambda_max_idi(i)=(1+r_ido(i)^2+sqrt(1+r_ido(i)^4+2*r_ido(i)^2*cos(2*theta_id_i(i))))/2;   
         % the lower bound inverse of the cond
             low_inv_cond(i)=sqrt(lambda_min_idi(i)/(lambda_max_idi(i)+u_max^2/md_id_o^2));     %md_id_o        
             if low_inv_cond(i)>low_inv_cond_id
                low_inv_cond_id=low_inv_cond(i);
                pair_id=i;
             else
             end
    end
    
end