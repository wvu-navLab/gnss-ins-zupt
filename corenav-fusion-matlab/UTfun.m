function [chi_UT_est,chi_UT_est_cov] = UTfun(mean,sigma)
%mean = slip value from the GP 
%sigma= cov bound

%% Transformation matrix example for cartesian to polar
% f(x,y) -> [r,theta], r=sqrt(x^2+y^2), theta=atan(y/x)
%% Transformation matrix for slip to odometry
% i=1-v_ins/wR -> w=v_ins/(R*(1-i))
chi0_slip=mean;
chi1_slip=mean+sigma; %(chi1=mean+(sqrt((n+lambda)Epsilon))
chi2_slip=mean-sigma; %(chi2=mean-(sqrt((n+lambda)Epsilon))

chi0_odo=0.8/(1-chi0_slip);
chi1_odo=0.8/(1-chi1_slip);
chi2_odo=0.8/(1-chi2_slip);
% chi0_odo=0.3*(1-chi0_slip);
% chi1_odo=0.3*(1-chi1_slip);
% chi2_odo=0.3*(1-chi2_slip);

chi_UT_est=(chi0_odo+chi1_odo+chi2_odo)/3;
chi_UT_est_cov=((chi0_odo-chi_UT_est)^2+(chi1_odo-chi_UT_est)^2+(chi2_odo-chi_UT_est)^2)/3;

end

