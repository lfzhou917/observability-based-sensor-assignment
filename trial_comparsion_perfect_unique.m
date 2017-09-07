clear all;      % clears all variables in your workspace
min_num_targets  = 1;
max_num_targets = 20; %set the target from 
num_trials = 30;
uo_max=1; % maximum motion ability for the target
for M = min_num_targets : max_num_targets % start from 1 target to 30 targets
     for i = 1  : num_trials % 100 trials
            pr=zeros(2*M,2); % the position of sensor
            pt=zeros(M,2); % position of  target
            pr(:,1)=100.*rand(2*M,1);
            pr(:,2)=100.*rand(2*M,1);
            pt(:,1)=100.*rand(M,1);
            pt(:,2)=100.*rand(M,1);
            cla;
            [perfect_value(i,M),unique_value(i,M)] = compare_perfect_unique_fun(M,uo_max,pr,pt);
     end
end
for M = min_num_targets : max_num_targets
avgperfect_value(M) = mean(perfect_value(:,M));
avgunique_value(M) = mean(unique_value(:,M));
stdperfect_value(M) = std(perfect_value(:,M));
stdunique_value(M) = std(unique_value(:,M));
end

figure; hold on;
errorbar(min_num_targets:max_num_targets,avgperfect_value(min_num_targets:max_num_targets),...
    stdperfect_value(min_num_targets:max_num_targets), 'r');
errorbar(min_num_targets:max_num_targets,avgunique_value(min_num_targets:max_num_targets),...
    stdunique_value(min_num_targets:max_num_targets),'b');
errorbar(min_num_targets:max_num_targets,1/3*avgperfect_value(min_num_targets:max_num_targets),...
    stdperfect_value(min_num_targets:max_num_targets), 'r');
title('Comparison of Perfect Pair and Unique Pair Assignment')
legend('Perfect Pair Assignment','Unique Pair Assignment','1/3 Perfect Pair Assignment');
xlabel('Number of targets');
ylabel('Total Reward');