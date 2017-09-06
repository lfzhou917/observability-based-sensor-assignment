%multi-robot-target tracking based on the observability metric
clc

N=4; % the number of robot
M=5; % the number of target

pr=zeros(N,2); % the position of robot
pt=zeros(M,2); % position of  target

% a simple case to test the best matching
pr=[-3 2; 0 0; 1 -1; 4 0];
pt=[-2 0; 1 2; 3 1; 5 -1; -1 7];

% control input of target
uo=[1 2];

% % plot the robot and the target
% figure(1);
% 
% 
% plot (pr(:,1), pr(:,2), 'kd', 'MarkerSize',10), hold on
% plot (pt(:,1), pt(:,2), 'rp', 'MarkerSize',10), hold on
% 
% axis([-4 6 -2 8]);

% plot (pr(1,1), pr(1,2), 'kd', 'MarkerSize',10), hold on
% txt1 = 's1';
% text(pr(1,1)-0.3, pr(1,2),txt1);
% 
% plot (pr(2,1), pr(2,2), 'kd','MarkerSize',10), hold on
% txt1 = 's2';
% text(pr(2,1)+0.2, pr(2,2),txt1);
% plot (pr(3,1), pr(3,2), 'kd','MarkerSize',10), hold on
% txt1 = 's3';
% text(pr(3,1), pr(3,2)+0.3,txt1);
% 
% plot (pt(1,1), pt(1,2), 'r*', 'MarkerSize',10), hold on
% txt1 = 't1';
% text(pt(1,1), pt(2,2)+1.3,txt1);
% 
% plot (pt(2,1), pt(2,2), 'r*','MarkerSize',10), hold on
% txt1 = 't2';
% text(pt(2,1), pt(2,2)+0.3,txt1);

%axis([-2 6 -2 6]);

uo_max=1; % the maximum control input of target, a prior known info.

% calculation of lower bounds of  inverse of condition number of the system
% including the control input, u_max

%ob_weight=invercond(pr,pt,uo_max,N,M);
ob_weight=invercond_u(pr,pt,uo,N,M);

% maximum matching
[val pairrobot_index target_index]=bipartite_matching(ob_weight);

% from the pairrobot_index and target_index to find 
% the best matching between pair and target
% establish match table, the first two are IDs of pair robots and the last
% is the ID of target
for i=1:size(pairrobot_index)
    match_table(i,:)=[fix((pairrobot_index(i)-1)/N)+1,mod(pairrobot_index(i)-1,N)+1,target_index(i)];
end


figure(2); clf; 
   axis equal; box on; hold on;
   axis([-4 6 -2 8]);
   plot (pr(:,1), pr(:,2), 'kd', 'MarkerSize',10), hold on
plot (pt(:,1), pt(:,2), 'rp', 'MarkerSize',10), hold on
for i=1:size(pairrobot_index)  
     plot([pr(match_table(i,1),1),pt(match_table(i,3),1)],[pr(match_table(i,1),2),pt(match_table(i,3),2)],'r:'), hold on
     plot([pr(match_table(i,2),1),pt(match_table(i,3),1)],[pr(match_table(i,2),2),pt(match_table(i,3),2)],'r:'), hold on
end
%    h(1) = covarianceEllipse([tarx_hat(1,k);tary_hat(1,k)],Sigma_hat(1:2,(2*k-1):2*k),[1 0 0],11.82);
%    h(2)=plot(tarx_hat(1,k),tary_hat(1,k),'rs','MarkerSize',8);
%    h(3)=plot(tarx_hat(1,1:k),tary_hat(1,1:k), 'r-');
%    h(4)=plot(tarxtrue(1,k),tarytrue(1,k),'bp','MarkerSize',8);
%    h(5)=plot(tarxtrue(1,1:k),tarytrue(1,1:k),'b-');
%    h(6)=plot(px,py,'kd','MarkerSize',8);
%    h(7)=plot ([px(id) tarx_hat(1,k)], [py(id) tary_hat(1,k)],':');
   %h(8)=plot ([px(pair_id(1,k)) tarx_hat(1,k)], [py(pair_id(1,k)) tary_hat(1,k)],':');
   legend('Sensor', 'Target');
   pause(0.1);

% %display the bipartite graph
%         s = [1 1 1 1 1 2 2 2 2 2 3 3 3 3 3 4 4 4 4 4 ...
%             5 5 5 5 5 6 6 6 6 6 7 7 7 7 7 8 8 8 8 8 ...
%             9 9 9 9 9 10 10 10 10 10 11 11 11 11 11 12 12 12 12 12 ...
%             13 13 13 13 13 14 14 14 14 14 15 15 15 15 15 16 16 16 16 16];
%         %t = [10 11 10 11 10 11 10 11 10 11 10 11 10 11 10 11 10 11];
%         t = [17 18 19 20 21 17 18 19 20 21 17 18 19 20 21 17 18 19 20 21 ...
%             17 18 19 20 21 17 18 19 20 21 17 18 19 20 21 17 18 19 20 21 ...
%             17 18 19 20 21 17 18 19 20 21 17 18 19 20 21 17 18 19 20 21 ...
%             17 18 19 20 21 17 18 19 20 21 17 18 19 20 21 17 18 19 20 21];
%         weights = [ob_weight(1,:), ob_weight(2,:), ob_weight(3,:),...
%                         ob_weight(4,:), ob_weight(5,:), ob_weight(6,:),...
%                         ob_weight(7,:), ob_weight(8,:), ob_weight(9,:),...
%                         ob_weight(10,:), ob_weight(11,:),ob_weight(12,:),...
%                         ob_weight(13,:), ob_weight(14,:),ob_weight(15,:),...
%                         ob_weight(16,:)];
%         names = {'s1' '(s1,s2)' '(s1,s3)' '(s1,s4)', '(s2,s1)' 's2' '(s2,s3)' '(s2,s4)',...
%             '(s3,s1)' '(s3,s2)' 's3' '(s3,s4)', '(s4,s1)' '(s4,s2)' '(s4,s3)' 's4','t1', 't2', 't3','t4', 't5'};
         
        %display the bipartite graph
        s = [1 1 1 1 1 2 2 2 2 2 3 3 3 3 3 4 4 4 4 4 ...
             5 5 5 5 5 6 6 6 6 6 7 7 7 7 7 ...
             8 8 8 8 8 9 9 9 9 9 10 10 10 10 10];
        %t = [10 11 10 11 10 11 10 11 10 11 10 11 10 11 10 11 10 11];
        t = [11 12 13 14 15 11 12 13 14 15 11 12 13 14 15 11 12 13 14 15 ...
             11 12 13 14 15 11 12 13 14 15 11 12 13 14 15 ...
             11 12 13 14 15 11 12 13 14 15 ...
             11 12 13 14 15];
        weights = [ob_weight(1,:), ob_weight(2,:), ob_weight(3,:),...
                        ob_weight(4,:), ob_weight(6,:),...
                        ob_weight(7,:), ob_weight(8,:),...
                         ob_weight(11,:),ob_weight(12,:),...
                        ob_weight(16,:)];
        names = {'s1' '(s1,s2)' '(s1,s3)' '(s1,s4)', 's2' '(s2,s3)' '(s2,s4)',...
             's3' '(s3,s4)', 's4','t1', 't2', 't3','t4', 't5'};

        %st=[3 6];
        %tt=[11 10];
        %T=graph(st,tt);
        G = graph(s,t,weights,names);
        G.Edges
        G.Nodes
        figure (3)
        h=plot(G,'EdgeLabel',G.Edges.Weight);
        %highlight(h,T,'EdgeColor','r','LineWidth',1.5);
        h.XData(1:10) = 1;
        h.XData(11:end) = 2;
        h.YData(1:10) = linspace(0,10,10);
        h.YData(11:end) = linspace(0,10,5);