function [P_new]= odomCovUpt(insAtt_old,P_old,insAtt_older,dt_odom,H11,H12,H21,H31,H32,H41,H42)

Cn2bUnc = eulr2dcm(insAtt_old);
%% Measurement Matrix members 
H11=-H11/dt_odom; %  H11= H11+ [1,0,0]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
H12=-H12/dt_odom;%   H12= H12+ [1,0,0]*Cn2bPlus*dtIMU;
H21= H21*(insAtt_old(3)-insAtt_older(3))/dt_odom^2; %  H21= H21+ sin(insAtt(2,i))*[0,cos(insAtt(1,i)),sin(insAtt(1,i))]*Cn2bPlus*dtIMU;
H31=-H31/dt_odom;%  H31= H31+ [0,1,0]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
H32=-H32/dt_odom; % H32= H32+ [0,1,0]*Cn2bPlus*dtIMU;
H24=-(cos(insAtt_old(2))*[0,0,1]*Cn2bUnc')/dt_odom; 
H41=-H41/dt_odom; %   H41= H41+ [0,0,1]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
H42=-H42/dt_odom;%         H42= H42+ [0,0,1]*Cn2bPlus*dtIMU;

%% Measurement Matrix 
H=[H11, H12, zeros(1,3) zeros(1,3) zeros(1,3);
    H21, zeros(1,3) zeros(1,3) H24, zeros(1,3);
    H31, H32, zeros(1,3), zeros(1,3) zeros(1,3);
    H41, H42, zeros(1,3), zeros(1,3) zeros(1,3)];
%% Measurement Noise Covariance Matrix
R=sigmaGP;
%% Kalman Gain
K=P_old*H'*inv(H*P_old*H'+R);
%% Update P matrix
P_new=(1 - K*H)*P_old*(1-K*H)' + K*R*K';

end