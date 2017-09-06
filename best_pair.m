function [pair_id,low_inv_cond_ji]=best_pair(N,px,py,tarx_hat,tary_hat,Sigma_hat)%Sigma_hat

% at every timestep give the best pair among all the sensors

% the maximum bound of the control input 
global u_max

% % calculate the distance between every robot and the target
%  d_io=zeros(N,1);
%  d_jo=zeros(N,1);

% %the Mahalanobis distance
     md_io=zeros(N,1);
% %the Mahalanobis distance
     md_jo=zeros(N,1);

%the ratio of d_i_o/d_id_o
r_ji=zeros(N,N);
%the angle between  id and i, w.r.t. target o 
theta_ji=zeros(N,N);


% the euclidean distance between the robot id and the targt 
%d_id_o=sqrt((px(id)-tarx_hat)^2+(py(id)-tary_hat)^2);

% % % the Mahalanobis distance
%  md_id_o=sqrt([(px(id)-tarx_hat);(py(id)-tary_hat)]'*inv(Sigma_hat)*[(px(id)-tarx_hat);(py(id)-tary_hat)]);


% maximum and minimum eigenvalue
lambda_max_ji=zeros(N,N);
lambda_min_ji=zeros(N,N);

% the lower bound inverse of the cond
low_inv_cond=zeros(N,N);

% give initial lower bound for pair ji;
low_inv_cond_ji=0;

% calculate the lower bound of the inverse of cond
% compare to select the maximum
    for i=1:N      
   
         for j=1:N            
           % calculate euclidean distance between every robot and the target
 %           d_io(i)=sqrt((px(i)-tarx_hat)^2+(py(i)-tary_hat)^2);   
%            % the Mahalanobis distance
            md_io(i)=sqrt([(px(i)-tarx_hat);(py(i)-tary_hat)]'*...
              inv(Sigma_hat)*[(px(i)-tarx_hat);(py(i)-tary_hat)]);
           % calculate euclidean distance between every robot and the target
 %           d_jo(j)=sqrt((px(j,1)-tarx_hat)^2+(py(j,1)-tary_hat)^2);   
%            % the Mahalanobis distance
             md_jo(j)=sqrt([(px(j)-tarx_hat);(py(j)-tary_hat)]'*...
              inv(Sigma_hat)*[(px(j)-tarx_hat);(py(j)-tary_hat)]);
% %          % the ratio of d_i_o/d_id_o or 
              r_ji(i,j)=md_jo(j)/md_io(i);
 %           r_ji(i,j)=d_jo(j)/d_io(i);
         % the angle between  id and i, w.r.t. target o 
             theta_ji(i,j)=atan2(norm(cross([(px(i)-tarx_hat),(py(i)-tary_hat),0],...
                     [(px(j)-tarx_hat),(py(j)-tary_hat),0])),dot([(px(i)-tarx_hat),(py(i)-tary_hat),0],...
                     [(px(j)-tarx_hat),(py(j)-tary_hat),0]));
                 
         %maximum and minimum eigenvalues of observability matrix. 
             lambda_min_ji(i,j)=(1+r_ji(i,j)^2-sqrt(1+r_ji(i,j)^4+2*r_ji(i,j)^2*cos(2*theta_ji(i,j))))/2;
             lambda_max_ji(i,j)=(1+r_ji(i,j)^2+sqrt(1+r_ji(i,j)^4+2*r_ji(i,j)^2*cos(2*theta_ji(i,j))))/2;
         % the lower bound inverse of the cond
             low_inv_cond(i,j)=sqrt(lambda_min_ji(i,j)/(lambda_max_ji(i,j)+u_max^2/md_io(i)^2));     %md_id_o        
             if low_inv_cond(i,j)>low_inv_cond_ji
                low_inv_cond_ji=low_inv_cond(i,j);
                pair_id=[i,j];
             else
             end
        end   
    end
    
end