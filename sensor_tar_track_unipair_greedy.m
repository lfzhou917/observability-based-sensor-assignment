% tracking by unique pair assignment
% based on the observability metric
% we assume the number of sensors is two time the number of targets
global u_max 

u_max=0.1; % for game %0.5 escape 2 2
%u_max=3; % for circle
N=8; %the number of sensor agents 
M = 3; % the number of targets
max_step=100; % loop time 315

Omega =zeros(1,M);
Omega(1)=10;
Omega(2)=20;
Omega(3)=-30;

C = {'r','b','m',[.5 .6 .7],[.8 .2 .6]};

pr =  [-3 0; -3 4; 0 0; 2 -1; 5 0; 4 6; 2 7; 1 3];

% at each timestep, we assign unique pair to each target, then based on these
% use EKF to eastimate the positions of the targets
r_pair_t_index =cell(1,max_step); %which pair assigned to which target
pt_hat=cell(1,max_step); % estimate of the target at each time step

pt_true = cell(1,max_step); % the true positions of the target
pt_true{1} = [2 1; -2 2; 3 4];

% thus we make up the pt _hat{1}
pt_hat{1} = [2.5 1.5; -0.5 0.5; 3.5 4.5];

%the covariance funciton 
t_Sigma_hat=cell(1, max_step);
t_Sigma_hat{1}=0.5*[1 0; 0 1; 1 0; 0 1; 1 0; 0 1];

%now, we use greed assignment to assign unique pair to each target at each
%time step
pt_x_store = cell(1,max_step);
pt_y_store = cell(1,max_step);

pt_x_store{1} = pt_true{1}(:,1)+1;
pt_y_store{1} = pt_true{1}(:,2)+0;


% Frobenius norm of covariance matrix.
normf_co=zeros(M,max_step);

% trace of covariance matrix.
co_trace=zeros(M,max_step);

%estimate error 
es_err=zeros(M,max_step);

% %Set up the movie.
writerObj = VideoWriter('multi_robot_multi_target_uniquepair.avi'); % Name it.
writerObj.FrameRate = 5; % How many frames per second.
open(writerObj); 
for k = 1: max_step
    % first obtain the assignment
    [~, ~, r_pair_t_index{k}] = compare_perfect_unique_fun(M,u_max,pr,pt_hat{k}); 
    % then use assignment to update the estimate of the targets
    for t = 1:M % for each target, update the estimate
       
        t_index = find(t==r_pair_t_index{k}(:,3)); %find which target, the row
        % then we know the index of associated two sensors
        s1_index= r_pair_t_index{k}(t_index,1); 
        s2_index= r_pair_t_index{k}(t_index,2);
      

[pt_hat{k+1}(t,1), pt_hat{k+1}(t,2), t_Sigma_hat{k+1}(2*t-1:2*t,:)] = KF (...
       pt_true{k}(t,1),pt_true{k}(t,2), pt_hat{k}(t,1), pt_hat{k}(t,2), t_Sigma_hat{k}(2*t-1:2*t,:),...
       [pr(s1_index,1);pr(s2_index,1)], [pr(s1_index,2);pr(s2_index,2)], [s1_index; s2_index], k);
   
   
       normf_co(t,k)=norm(t_Sigma_hat{k}(2*t-1:2*t,:),'fro');
       co_trace(t,k)=trace(t_Sigma_hat{k}(2*t-1:2*t,:)); %trace(Sigma_hat(1:2,(2*k+1): 2*(k+1)));
       es_err(t,k)=sqrt((pt_hat{k}(t,1)-pt_true{k}(t,1))^2+(pt_hat{k}(t,2)-pt_true{k}(t,2))^2);
   
   
   % afte the first update, the true position needs to be updated
       pt_true{k+1}(t,1) = cos(k/Omega(t)) + pt_true{1}(t,1);
       pt_true{k+1}(t,2) = sin(k/Omega(t)) + pt_true{1}(t,2);
       
       
       % for each target and sensor-pair, plot 
           figure(1);
           title('Greedy Unique Pair Assignment','fontsize',14)
           xlabel({'$$x$$'},'Interpreter','latex','fontsize',16)
           ylabel({'$$y$$'},'Interpreter','latex','fontsize',16)         
           axis equal; box on; hold on;
           xlim([-4 6]);
           ylim([-2 8]);
           h(1) = covarianceEllipse([pt_hat{k}(t,1);pt_hat{k}(t,2)],t_Sigma_hat{k}(2*t-1:2*t,:),C{t},11.82);
           h(2)=plot(pt_hat{k}(t,1),pt_hat{k}(t,2),'rs','MarkerSize',12,'color',C{t},'MarkerFaceColor',C{t});
           h(4)=plot(pt_true{k}(t,1),pt_true{k}(t,2),'bp','MarkerSize',12,'color',C{t});
           h(6)=plot(pt_x_store{k}(t,:),pt_y_store{k}(t,:),'b:','LineWidth',2,'color',C{t});
           h(7)=plot(pr(:,1),pr(:,2),'kd','MarkerSize',12);
           h(8)=plot ([pr(s1_index,1) pt_hat{k}(t,1)], [pr(s1_index,2) pt_hat{k}(t,2)],'-','color',C{t}, 'LineWidth',1.5);
           h(9)=plot ([pr(s2_index,1) pt_hat{k}(t,1)], [pr(s2_index,2) pt_hat{k}(t,2)],'-','color',C{t}, 'LineWidth',1.5);
           %legend('Target Covariance','Target Estimate Mean','True Target','True Target Trajectory','Sensor','Location','northwest');
           
    end
    pause(0.01);
    frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
    clf;
    pt_x_store{k+1} = horzcat(pt_x_store{k}, pt_true{k+1}(:,1));
    pt_y_store{k+1} = horzcat(pt_y_store{k}, pt_true{k+1}(:,2));
end
      
      hold  off
  close(writerObj); % Saves the movie.
  %plot three target's mean error and trace(covariance)
           figure(2); clf; 
           axis equal; box on; hold on;
           
           subplot(3,1,1)
           h(1)=plot(es_err(1,:));
           
           subplot(3,1,2) 
           h(2)=plot(es_err(2,:));
           
           subplot(3,1,3)
           h(3)=plot(es_err(3,:));

           
           figure(3); clf; 
           axis equal; box on; hold on;
           
           subplot(3,1,1)
           h(1)=plot(co_trace(1,:));
           
           subplot(3,1,2) 
           h(2)=plot(co_trace(2,:));
           
           subplot(3,1,3)
           h(3)=plot(co_trace(3,:));