function [ob_weight_singlepair] = invercond_u(pr,pt,u,N,M)
%here is the target's view, the target knows the its own control input...
% so that it can calculate the exact inverse of cond. 
ob_weight_singlepair=zeros(N*N,M); 
for i=1:N
    for j=1:N 
             for k=1:M                 
                 %robot pair i, j and target k
                 if i ~= j
                     ob_matrix=[(pt(k,1) - pr(i,1)), (pt(k,2) - pr(i,2));...
                         (pt(k,1) - pr(j,1)), (pt(k,2) - pr(j,2));...
                         u(1), u(2)];
                     ob_weight_singlepair(N*(i-1)+j,k)=1/(cond(ob_matrix));
                 else 
                    ob_matrix=[(pt(k,1) - pr(i,1)), (pt(k,2) - pr(i,2));...
                         u(1), u(2)];
                     ob_weight_singlepair(N*(i-1)+j,k)=1/(cond(ob_matrix));
                 end
             end     
    end
end

end