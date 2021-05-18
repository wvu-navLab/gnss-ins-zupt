%% ZARU - Zero Angular Rate Update
% Zero angular rate may be assumed whenever the vehicle is stationary
% so ZARU may be performed whenever a ZUPT performed.
% -----------------------------------------------------
% When odometry is available it may be used to trigger a ZARU when the vehicle is
% stationary. 
% -----------------------------------------------------
% ZARU may also be performed when the vehicle is moving at a
% constant heading. This may be detected by comparing three parameters with
% thresholds: 
% the standard deviation of the recent yaw-rate gyro measurements, 
% the standard deviation of the steering-angle commands, 
% yaw rate obtained from differential odometry.
% -----------------------------------------------------
%
function [insVel_new,insAtt_new,insLLH_new,x_err_new,P_new,postFit] = zeroUpd(insVel_old,insAtt_old,insLLH_old,x_err_old,P_old,omega_b_ib)
    Cn2b_zaru= eulr2dcm(insAtt_old);
    z_zaru=-omega_b_ib';
    z_zupt=-insVel_old;
    H_zaru=[zeros(3), zeros(3), zeros(3), zeros(3), -eye(3)];
    H_zupt=[zeros(3), -eye(3), zeros(3), zeros(3), zeros(3)];
    H_tot=[H_zaru;H_zupt];
    R_zaru=[0.01^2, 0.0,   0.0;
        0.0,   0.01^2, 0.0;
        0.0,   0.0,   0.0025^2];
    R_zupt=[0.02^2, 0.0,   0.0;
        0.0,   0.02^2, 0.0;
        0.0,   0.0,   1.0^2];
    
    R_tot=eye(6).*[diag(R_zaru);diag(R_zupt)];
    K_zaru=P_old*H_tot'*inv(H_tot*P_old*H_tot'+R_tot);
    x_err_new=x_err_old+K_zaru*([z_zaru;z_zupt]-H_tot*x_err_old);
    postFit.z=z_zaru;
    postFit.H=H_zaru;
    insAtt_new=dcm2eulr((eye(3)-skewsymm(x_err_new(1:3)))*Cn2b_zaru');
    insVel_new=insVel_old-x_err_new(4:6);
    insLLH_new=insLLH_old-x_err_new(7:9);
    x_err_new(1:9)=zeros(1,9);
    P_new=(eye(15) - K_zaru*H_tot)*P_old*(eye(15)-K_zaru*H_tot)' + K_zaru*R_tot*K_zaru';
end
