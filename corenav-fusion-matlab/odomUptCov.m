function [P_new]= odomUptCov(insAtt_old,P_old,insAtt_older,dt_odom,H11,H12,H21,H31,H32,H41,H42,T_r,sigma_cmc)

Cn2bUnc = eulr2dcm(insAtt_old);
%% Measurement Matrix members --
H11=-H11/dt_odom;
H12=-H12/dt_odom;
H21=H21*(insAtt_old(3)-insAtt_older(3))/dt_odom^2;
H31=-H31/dt_odom;
H32=-H32/dt_odom;
H24=-(cos(insAtt_old(2))*[0,0,1]*Cn2bUnc')/dt_odom;
H41=-H41/dt_odom;
H42=-H42/dt_odom;


%% Measurement Matrix -- Checked
H=[H11, H12, zeros(1,3) zeros(1,3) zeros(1,3);
    H21, zeros(1,3) zeros(1,3) H24, zeros(1,3);
    H31, H32, zeros(1,3), zeros(1,3) zeros(1,3);
    H41, H42, zeros(1,3), zeros(1,3) zeros(1,3)];
%% Measurement Noise Covariance Matrix
R= [0.5,   0.5,   0,0;
    1/T_r,-1/T_r, 0,0;
    0,     0,     1,0;
    0,0,0,1]* ...
    [sigma_R^2, 0,            0,0;
    0,            sigma_R^2,  0,0;
    0,             0,            sigma_cmc^2,0;
    0,0,0,0.05^2]* ...
    [0.5,  1/T_r, 0,0;
    0.5, -1/T_r, 0,0;
    0,    0,     1,0;
    0,0,0,1];

%% Kalman Gain
K=P_old*H'*inv(H*P_old*H'+R);

%% Update P matrix
P_new=(eye(15) - K*H)*P_old*(eye(15)-K*H)' + K*R*K';

end