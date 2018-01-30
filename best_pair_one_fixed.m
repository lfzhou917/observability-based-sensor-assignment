% tracking by using best pair 
% based on the observability metric
global u_max
%global tarx_true tary_true


%u_max=0.1; % for game %0.5 escape 2 2
u_max=3; % for circle
N=6; %the number of sensor agents 
max_step=315; % loop time 315

% the states (px, py) of N sensor robots
% px=zeros(N,1);
% py=zeros(N,1);

px=[-3; 0; 1; 4; 4; 2];
py=[2; 0;  -1; 0; 4; 5];

% px=[-6; -2; 1; 8; 8; 2];
% py=[2; -2;  -4; 0; 8; 10];

% the state of the best pair
pair_id=zeros(max_step,2);
% lower bound of the inverse of cond from sensor side
low_inv_cond_id=zeros(1,max_step);
% inverse of cond from target
inv_cond_tar=zeros(1,max_step);
% robot_id
id=2;
id_sensor=zeros(1,max_step);
% define the neighbor of id
id_ne=zeros(N,1);
    if(id<N)
    id_ne(id)=id+1;
    else
    id_ne(id)=1;
    end

% true postion of target
tarx_true=zeros(1,max_step);
tary_true=zeros(1,max_step); 
%  tarx_true(1,1)=2; %4 4
%  tary_true(1,1)=2; %for game 2,1 2, 3 -2 3 -1 4 -2 7
 tarx_true(1,1)=5;
 tary_true(1,1)=3;% for circle

%estimation of the target
tarx_hat=zeros(N,max_step);
tary_hat=zeros(N,max_step);
tarx_hat(1,1)=5.5; %%3.5 3.5
tary_hat(1,1)=3.5;

%the covariance funciton 
Sigma_hat=zeros(2*N,2*max_step);
Sigma_hat(1:2,1:2)=0.5*[1 0; 0 1];

%the control selction of the target
ux_op=zeros(1,max_step);
uy_op=zeros(1,max_step);

% Frobenius norm of covariance matrix.
normf_co=zeros(1,max_step);

% trace of covariance matrix.
co_trace=zeros(1,max_step);

%estimate error 
es_err=zeros(1,max_step);

%Set up the movie.
writerObj = VideoWriter('best_pair_one_fixed.avi'); % Name it.
writerObj.FrameRate = 5; % How many frames per second.
open(writerObj); 

for k=1:max_step
        %calculate the bset pair
        id_sensor(1,k)=id;
        [pair_id(1,k),low_inv_cond_id(1,k)]=pair_assign_observability(N,id,px,py,...
            tarx_hat(1,k),tary_hat(1,k),Sigma_hat(1:2,(2*k-1): 2*k)); %Sigma_hat(1:2,(2*k-1): 2*k)
   
     %estimate of the target best pair
        [tarx_hat(1,k+1), tary_hat(1,k+1), Sigma_hat(1:2, (2*(k+1)-1):(2*(k+1)))] = KF (...
       tarx_true(1,k),tary_true(1,k), tarx_hat(1,k), tary_hat(1,k), Sigma_hat(1:2,(2*k-1): 2*k),...
       [px(id);px(pair_id(1,k))], [py(id);py(pair_id(1,k))], [id;pair_id(1,k)], k);
       
        %calculate the with next neighbor
      %  [low_inv_cond_id(1,k)]=next_assign_observability(id,1,px,py,tarx_hat(1,k),tary_hat(1,k),Sigma_hat(1:2,(2*k-1): 2*k));
       % estimate of the target best pair, next neighbor
%         [tarx_hat(1,k+1), tary_hat(1,k+1), Sigma_hat(1:2, (2*(k+1)-1):(2*(k+1))), tarx_true(1,k), tary_true(1,k)] = KF (...
%         tarx_true(1,k),tary_true(1,k), tarx_hat(1,k), tary_hat(1,k), Sigma_hat(1:2,(2*k-1): 2*k),...
%         [px(id);px(id_ne(id))], [py(id);py(id_ne(id))], [id;id_ne(id)], k);     

      
%         [tarx_hat(1,k+1), tary_hat(1,k+1), Sigma_hat(1:2, (2*(k+1)-1):(2*(k+1)))] = KF (...
%         tarx_true(1,k),tary_true(1,k), tarx_hat(1,k), tary_hat(1,k), Sigma_hat(1:2,(2*k-1): 2*k),...
%         [px(id);px(1)], [py(id);py(1)], [id;1], k); 

%    % single robot
%     [tarx_hat(1,k+1), tary_hat(1,k+1), Sigma_hat(1:2, (2*(k+1)-1):(2*(k+1))), tarxtrue(1,k), tarytrue(1,k)] = KF (...
%     tarx_hat(1,k), tary_hat(1,k), Sigma_hat(1:2,(2*k-1): 2*k), px(id), py(id), id, k);   

           [tarx_true(k+1), tary_true(k+1)]=target_motion(tarx_true(1,1), tary_true(1,1), k);
       
           %normf_co(1,k)=norm(Sigma_hat(1:2,(2*k-1): 2*k),'fro');
           co_trace(1,k)=trace(Sigma_hat(1:2,(2*k-1): 2*(k))); %trace(Sigma_hat(1:2,(2*k+1): 2*(k+1)));
%            min_eig(k)=min(eig(Sigma_hat(1:2,(2*k-1): 2*(k))));
%            max_eig(k)=max(eig(Sigma_hat(1:2,(2*k-1): 2*(k))));
           es_err(1,k)=sqrt((tarx_hat(1,k)-tarx_true(1,k))^2+(tary_hat(1,k)-tary_true(1,k))^2);

           figure(1); clf; 
           title('Best Pair for One Fixed Sensor s2','fontsize',14)
           %title('Best pairing for fixed sensor s_2','fontsize',14)
           xlabel({'$$x$$'},'Interpreter','latex','fontsize',16)
           ylabel({'$$y$$'},'Interpreter','latex','fontsize',16)         
           axis equal; box on; hold on;
           xlim([-4 6]);
           ylim([-2 8]);
%        xlim([-7 10]);
%        ylim([-6 12]);
           h(1) = covarianceEllipse([tarx_hat(1,k);tary_hat(1,k)],Sigma_hat(1:2,(2*k-1):2*k),[1 0 0],11.82);
           h(2)=plot(tarx_hat(1,k),tary_hat(1,k),'rs','MarkerSize',12,'MarkerFaceColor','r');
           %h(3)=plot(tarx_hat(1,1:k),tary_hat(1,1:k), 'r-');
           h(4)=plot(tarx_true(1,k),tary_true(1,k),'bp','MarkerSize',12);
           h(6)=plot(tarx_true(1,1:k),tary_true(1,1:k),'b:','LineWidth',2);
           %(5)=quiver(tarx_true(1,k),tary_true(1,k),ux_op(1,k),uy_op(1,k));
           h(7)=plot(px,py,'kd','MarkerSize',12);
%            txt1 = 's_1';
%            text(-3,2.5,txt1)
            txt1 = 's_2';
            text(-0.5,0,txt1,'LineWidth',2)
            h(8)=plot ([px(id) tarx_hat(1,k)], [py(id) tary_hat(1,k)],'r-','LineWidth',2);
            h(9)=plot ([px(pair_id(1,k)) tarx_hat(1,k)], [py(pair_id(1,k)) tary_hat(1,k)],'r-','LineWidth',2);
       
 %          h(9)=plot ([px(1) tarx_hat(1,k)], [py(1) tary_hat(1,k)],':');
           legend('Target Covariance','Target Estimate Mean','True Target','True Target Trajectory','Sensor','Location','northwest');
           pause(0.01);
           

%            figure(2); clf; 
%            axis equal; box on; hold on;
%            
%            subplot(4,1,1)
%            h(1)=plot(low_inv_cond_id(1:k));
%            
%            subplot(4,1,2) 
%            h(2)=plot(inv_cond_tar(1:k));
%            
%            subplot(4,1,3)
%            h(3)=plot(es_err(1:k));
%            
%            subplot(4,1,4)
%            %h(3)=plot(normf_co(1,1:k));
%            h(4)=plot(co_trace(1,1:k));
%            %plot(min_eig(1:k),'r'), hold on
%            %plot(max_eig(1:k),'b')
                      
%            subplot(3,1,1)
%            h(1)=plot(low_inv_cond_id(1:k));
%            
%            subplot(3,1,2)
%            h(3)=plot(es_err(1:k));
%            
%            subplot(3,1,3)
%            %h(3)=plot(normf_co(1,1:k));
%            h(4)=plot(co_trace(1,1:k));
% 
%            figure(3); clf; 
%            box on; hold on;
%            ylim([0 7]);
%            plot(pair_id(1,1:k))
%            plot(id_sensor(1,1:k))
          
%          plot(pair_id(1:k,:));
%            subplot(2,1,1)
%            h1=plot(pair_id(1:k,1));
%            subplot(2,1,2)
%            h2=plot(pair_id(1:k,2));

 %///************************************************************   
 %if mod(i,4)==0, % Uncomment to take 1 out of every 4 frames.
        frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
        writeVideo(writerObj, frame);
    %end
end
hold  off
close(writerObj); % Saves the movie.