%compare the perfect pair assignment and unique pair assignment with 
% the lower bound of  inverse of the cond number
N=2; % the number of sensors
M=1; % the number of targets
uo_max=1; % maximum motion ability for the target

pr=zeros(N,2); % the position of sensor
pt=zeros(M,2); % position of  target
pr(:,1)=100.*rand(N,1);
pr(:,2)=100.*rand(N,1);
pt(:,1)=100.*rand(M,1);
pt(:,2)=100.*rand(M,1);

low_invcond=invercond(pr,pt,uo_max,length(pr),length(pt));
% perfect assignment
[perfectvalue, pairsensor_index, target_index]=bipartite_matching(low_invcond);
perfect_eachstep=[];
for i=1:size(pairsensor_index)
    perfect_eachstep=[perfect_eachstep; low_invcond(pairsensor_index(i),target_index(i))];
end
% unique assignment
unique_eachstep=zeros(M,1);
s_selected=zeros(M,2);
for t=1:M
unique_eachstep(t)=max(max(low_invcond));
[row_max,col_max]=find(low_invcond==unique_eachstep(t)); % how to make it unique??
pt(col_max(1),:)=[]; % delete the target has been tracked

s_selected(t,1)=fix((row_max(1)-1)/length(pr))+1;
s_selected(t,2)=mod(row_max(1)-1,length(pr))+1;% find the sensor index
pr([s_selected(t,1) s_selected(t,2)],:)=[];
%pr(s_selected(t,2),:)=[]; %delete the sensor pair has been used. 

low_invcond=invercond(pr,pt,uo_max,length(pr(:,1)),length(pt(:,1)));
end
unique_value=sum(unique_eachstep);
appro_ratio=unique_value/perfectvalue; 