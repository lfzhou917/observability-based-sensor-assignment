function [low_inv_cond_id]=next_assign_observability(id,id_ne,px,py,tarx_hat,tary_hat,Sigma_hat)
              global u_max
               % the maximum bound of the control input 
%                 % the distance between the robot id and the targt 
%                 d_id_o=sqrt((px(id)-tarx_hat)^2+(py(id)-tary_hat)^2);
                
                % % the Mahalanobis distance
                md_id_o=sqrt([(px(id)-tarx_hat);(py(id)-tary_hat)]'*...
                    inv(Sigma_hat)*[(px(id)-tarx_hat);(py(id)-tary_hat)]);
%                  % the distance between the robot id and the targt 
%                 d_idnext_o=sqrt((px(id_ne)-tarx_hat)^2+(py(id_ne)-tary_hat)^2);
                md_idnext_o=sqrt([(px(id_ne)-tarx_hat);(py(id_ne)-tary_hat)]'*...
                    inv(Sigma_hat)*[(px(id_ne)-tarx_hat);(py(id_ne)-tary_hat)]);
                
                
                % the ratio of d_i_o/d_id_o
                r_ido=md_idnext_o/md_id_o;
                % the angle between  id and i, w.r.t. target o 
                 theta_id_i=atan2(norm(cross([(px(id)-tarx_hat),(py(id)-tary_hat),0],...
                 [(px(id_ne)-tarx_hat),(py(id_ne)-tary_hat),0])),dot([(px(id)-tarx_hat),(py(id)-tary_hat),0],...
                 [(px(id_ne)-tarx_hat),(py(id_ne)-tary_hat),0]));
                 
                %maximum and minimum eigenvalues of observability matrix. 
                lambda_min_idi=(1+r_ido^2-sqrt(1+r_ido^4+2*r_ido^2*cos(2*theta_id_i)))/2;
                lambda_max_idi=(1+r_ido^2+sqrt(1+r_ido^4+2*r_ido^2*cos(2*theta_id_i)))/2;   
                % the lower bound inverse of the cond
                low_inv_cond_id=sqrt(lambda_min_idi/(lambda_max_idi+u_max^2/md_id_o^2));      
end