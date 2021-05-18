function [insAtt_new,insVel_new,insLLH_new,x_err_new,P_new,postFit]= odomUpt(insVel_old,insAtt_old,insLLH_old,x_err_old,P_old,insAtt_older,heading_old,dt_odom,rearVel,headRate,s_or,H11,H12,H21,H31,H32,H41,H42,z11,z21,z31,z41,T_r,sigma_or_L,sigma_or_R,sigma_cmc,s_delta_or)

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

%% Measurement Innovation -- Odometry eq:16.42 --
z1_odom=rearVel*(1-s_or);
z2_odom=(headRate)*(1-s_or)-((z11/dt_odom)/T_r)*s_delta_or;

%% Measurement Innovation w/o time -- INS eq:16.42 --
z1_ins=z11; %Checked
z2_ins=((insAtt_old(3)-heading_old))*z21;
if abs(z2_ins) > 0.5
    disp('Heading spike! errorCorrected')
    z2_ins=0;
end
z3_ins=z31; %Checked
%% Measurement Innovation -- (z= Odom - INS) with zero cross-track eq:16.42  and z speed
z11=z1_odom-z1_ins/dt_odom;
z21=z2_odom-z2_ins/dt_odom^2;
z31=0.0-z31/dt_odom;
z41=0.0-z41/dt_odom;

%% Measurement Matrix -- Checked
H=[H11, H12, zeros(1,3) zeros(1,3) zeros(1,3);
    H21, zeros(1,3) zeros(1,3) H24, zeros(1,3);
    H31, H32, zeros(1,3), zeros(1,3) zeros(1,3);
    H41, H42, zeros(1,3), zeros(1,3) zeros(1,3)];
%% Measurement Noise Covariance Matrix
R= 25*[0.5,   0.5,   0,0;
    1/T_r,-1/T_r, 0,0;
    0,     0,     1,0;
    0,0,0,1]* ...
    [sigma_or_L^2, 0,            0,0;
    0,            sigma_or_R^2,  0,0;
    0,             0,            sigma_cmc^2,0;
    0,0,0,0.05^2]* ...
    [0.5,  1/T_r, 0,0;
    0.5, -1/T_r, 0,0;
    0,    0,     1,0;
    0,0,0,1];

%% Kalman Gain
K=P_old*H'*inv(H*P_old*H'+R);

% C_deltaz=H*P_old*H'+R;
% CC=sqrt(diag(C_deltaz))
% sqrt(C_deltaz)

%% Update state estimation
x_err_new=(x_err_old+K*([z11;z21;z31;z41]-H*x_err_old));%;/sqrt(diag(C_deltaz));
postFit.z=[z11;z21;z31;z41];
postFit.H=H;
insAtt_new=dcm2eulr((eye(3)-skewsymm(x_err_new(1:3)))*Cn2bUnc');
insVel_new=(insVel_old-x_err_new(4:6));
insLLH_new=insLLH_old-x_err_new(7:9);
x_err_new(1:9)=zeros(1,9);

%% Update P matrix
P_new=(eye(15) - K*H)*P_old*(eye(15)-K*H)' + K*R*K';

% counter=counter+1;
% kk=kk+1;
end