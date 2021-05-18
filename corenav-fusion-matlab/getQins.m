%inputs(F21,CbnPlus, T_p_rn)

function [Qins]=getQins(F21,Cb2nPlus, insLLH, R_N,R_E,dt)
T_rn_p=[1.0/(R_N+insLLH(3)),0,0;
            0,1.0/((R_E+insLLH(3))*cos(insLLH(1))),0;
            0,0,-1];
Ts=dt;%1/200
g= 9.80665;
deg2rad = pi/180;

%% TODO -- check below values for the specific IMU::ADIS 16488
%eq 14.83,84 pg 592
%     init_accel_bias = 0;
%     init_gyro_bias = 0.7*deg2rad;
%     sig_accel_inRun = (3.2e-6*g); % m/s
%     sig_gyro_inRun = 0.8*4.848e-6 ; %rad/s
%     
%     sig_VRW = .008*sqrt(Ts); %m/s
%     sig_ARW = .09*2.909e-4*sqrt(Ts); %rad
% sig_gyro_inRun = 1.6*pi/180/3600;%          | sig_bgd::rad/s
% sig_ARW = .3*(pi/180)*sqrt(3600)/3600;%     | sig_rg ::rad/s
% sig_accel_inRun = (0.1e-3*g); %             | sig_bad::m/s^2
% sig_VRW = 0.029*sqrt(3600)/3600;%           | sig_ra ::m/s^2 !!
sig_gyro_inRun = 1.6*pi/180/3600;%          | sig_bgd::rad/s
sig_ARW = .1*(pi/180)*sqrt(3600)/3600;%     | sig_rg ::rad/s
sig_accel_inRun = (3.2e-6*g); %             | sig_bad::m/s^2
sig_VRW = 0.008*sqrt(3600)/3600;%           | sig_ra ::m/s^2 !!
    % following 14.2.6 of Groves pp 592
    Srg= (sig_ARW^2)*Ts;
    Sra= (sig_VRW^2)*Ts;
    
    Sbad=(sig_accel_inRun^2)/Ts;
    Sbgd=(sig_gyro_inRun^2)/Ts;

%%     
    
    %Use Navigation Frame, gamma=n, Q , eq 14.81
    Q11=(Srg*Ts+(1/3)*Sbgd*Ts^3)*eye(3); % Checked
    
    Q21=((1/2)*Srg*Ts^2+(1/4)*Sbgd*Ts^4)*F21; %Checked
    
    Q12=Q21';%Checked
    
    Q31=((1/3)*Srg*Ts^3+(1/5)*Sbgd*Ts^5)*T_rn_p*F21; %Checked
    
    Q13=Q31'; %Checked
    
    Q14=zeros(3); %Checked
    
    Q15=(1/2)*Sbgd*Ts^2*Cb2nPlus; %Checked
    
    Q22=(Sra*Ts+(1/3)*Sbad*Ts^3)*eye(3)+((1/3)*Srg*Ts^3+(1/5)*Sbgd*Ts^5)*(F21*F21'); %Checked
    
    Q32=((1/2)*Sra*Ts^2+(1/4)*Sbad*Ts^4)*T_rn_p+((1/4)*Srg*Ts^4+(1/6)*Sbgd*Ts^6)*T_rn_p*(F21*F21'); %Checked
    
    Q23=Q32'; %Checked
    
    Q24=(1/2)*Sbad*Ts^2*Cb2nPlus; %Checked
   
    Q25=(1/3)*Sbgd*Ts^3*F21*Cb2nPlus; %Checked
    
    Q33=((1/3)*Sra*Ts^3 + (1/5)*Sbad*Ts^5)*(T_rn_p*T_rn_p)+((1/5)*Srg*Ts^5+(1/7)*Sbgd*Ts^7)*T_rn_p*(F21*F21')*T_rn_p; %Checked
   
    Q34=(1/3)*Sbad*Ts^3*T_rn_p*Cb2nPlus; %Checked
   
    Q35=(1/4)*Sbgd*Ts^4*T_rn_p*F21*Cb2nPlus; %Checked
  
    Q41=zeros(3); %Checked
   
    Q42=(1/2)*Sbad*Ts^2*(Cb2nPlus)';%Cnb=Cbn' %Checked
   
    Q43=Q34'; %Checked
   
    Q44=Sbad*Ts*eye(3); %Checked
   
    Q45=zeros(3); %Checked
   
    Q51=(1/2)*Sbgd*Ts^2*(Cb2nPlus)'; %Checked
  
    Q52=(1/3)*Sbgd*Ts^3*F21'*(Cb2nPlus)'; %Checked
   
    Q53=Q35'; %Checked
  
    Q54=zeros(3); %Checked
   
    Q55=Sbgd*Ts*eye(3); %Checked
    
    
    
    Qins=[Q11, Q12, Q13, Q14, Q15;
          Q21, Q22, Q23, Q24, Q25;
          Q31, Q32, Q33, Q34, Q35;
          Q41, Q42, Q43, Q44, Q45;
          Q51, Q52, Q53, Q54, Q55];
      
    

%      P=blkdiag(eye(3)*(init_accel_bias^2)*Ts,eye(3)*(init_gyro_bias^2)*Ts);
%     
%      noiseChar = [Srg,Sra,Sbad,Sbgd];
%      
%      Pgyro=diag((ARW).^2);