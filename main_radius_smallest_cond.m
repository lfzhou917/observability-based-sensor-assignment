% main setup
s1=[0,1];
s2=[-sqrt(3)/2, -1/2];
s3=[sqrt(3)/2, -1/2];

sm_cond_r=radius_sm_cond(s1,s2,s3,0);
for cov_r=0.05:0.05:2
sm_cond_r=[sm_cond_r;radius_sm_cond(s1,s2,s3,cov_r)];
%plot(sm_cond_r(0:0.1:cov_r)), hold on
end