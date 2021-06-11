% Authors: Cagri Kilic and Jason Gross
% Date: September 2018
%%  -----------------------------------------------------------------------
% clc;
clear;
% load t10lastCN.mat%t11filt2.mat % t9filt2.mat
load t10lastCN.mat
load gpsVelShift3.mat
load vGPSX.mat
load gtsamRes.mat
for ss=1:length(gtsam.time)
llhGTSAM(:,ss)=xyz2llh([gtsam.X(ss);gtsam.Y(ss);gtsam.Z(ss)]);
end
% load gpsVelyOrig.mat
% gpsHeading
yaw=-42;
% t11:312
% t9:320
%t10:-42
%% initialize variables
MainRunInit2 
%% odometry calculations
odom
%% Double Low Pass Filter
% dema
for i=2:L

    dtIMU=tTimu(i)-tTimu(i-1); % calculates imu delta time from rostomat output tTimu
    TimeIMU(i)=dtIMU+TimeIMU(i-1); % creates imu time from delta time starting from zero
    %% IMU mounting orientation for BadgerRover
    omega_b_ib=[Gx(i),Gy(i),Gz(i)]-bg(:,i-1)'; %rad/s IMU gyro outputs minus estimated gyro bias
    f_ib_b = [Ax(i),Ay(i),Az(i)]-ba(:,i-1)'; %m/s^2 IMU acceleration minus estimated acce bias
    v_ib_b= f_ib_b* dtIMU; %m/s acceleration times delta IMU = velocity
    %% Attitude Update
    [insAttPlus, Cb2nPlus,Cb2nMinus,Omega_n_en,Omega_n_ie,R_N,R_E,omega_n_in,omega_n_ie] = AttUpdate(insAtt(:,i-1),omega_ie,insLLH(:,i-1),omega_b_ib,ecc,Ro,insVel(:,i-1),dtIMU);
    insAtt(:,i)=insAttPlus;
    %% Velocity Update
    [insVelPlus] = VelUpdate(Cb2nMinus, Cb2nPlus, v_ib_b,insVel(:,i-1),insLLH(:,i-1),omega_ie,Ro,ecc,dtIMU);
    insVel(:,i)=insVelPlus;
    %% Position Update
    [insLLHPlus,R_EPlus,r_eb_e] = PosUpdate(insLLH(:,i-1),insVel(:,i),insVel(:,i-1),Ro,ecc,dtIMU);
    insLLH(:,i)=insLLHPlus;
    insXYZ(:,i)=r_eb_e;
    %% Error State Model--eq 14.63
    [STM] = insErrorStateModel_LNF(R_EPlus,R_N,insLLH(:,i),insVel(:,i),dtIMU,Cb2nPlus,omega_ie,omega_n_in,f_ib_b);
    %% Propagation of the Error
    x_err=STM*x_err;
    %% Q matrix --
    F21= -skewsymm(Cb2nPlus*(f_ib_b'));
    Q=getQins(F21,Cb2nPlus,insLLH(:,i),R_N,R_E,dtIMU);
    %% P matrix
    P = STM*P*STM' + Q; 
    %% positiveDefiniteCheck of P and Q matrices
    positiveDefiniteCheck
    %% store fixed values before GP
    STM_fixed=STM;
    Q_fixed=Q;
    P_fixed=P;
    P_pred=P;
    %% Integrate IMU specific measurements for odometry update
    if odomUpdate()
        insIntegrationforOdomUpdate
    end
    %% NonHolonomic motion constraints as a Pseudo-Update
    if nonHolo()
        [insAtt(:,i),insVel(:,i),insLLH(:,i),x_err,P]= nonHolonomic(ang_z(kk),insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,omega_n_ie,omega_b_ib,A);
    end
    xState{1,i}=x_err;
    PStore{1,i}=P;
    STMStore{1,i}=STM+eye(15).*x_err;
    %% Check for wheel odometry update availability
    if kk<min(min(length(heading),length(lin_x))) % Only valid for post-processing
%         if tTimu(i)>=tTodom(kk) % Odometry update is available
        if TimeIMU(i)>=TimeODOM(kk) % Odometry update is available
            bb(counter)=i; % counts for the number of `i` when the loop goes into odometry updates
            dt_odom=tTodom(kk)-tTodom(kk-1);
            TimeODOM(kk)=dt_odom+TimeODOM(kk-1);
            %% Odometry Update
            if odomUpdate()
                odomUptCount=odomUptCount+1;
                P_old=P;
                insAtt_old= insAtt(:,i);
                insVel_old= insVel(:,i);
                insLLH_old= insLLH(:,i);
                x_err_old=x_err;
                [insAtt(:,i),insVel(:,i),insLLH(:,i),x_err,P,postFitOdom]= odomUpt(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,...
                    insAtt(:,i-1),insAtt(3,bb(counter-1)),dt_odom,rearVel(kk),headRate(kk),s_or,...
                    H11,H12,H21,H31,H32,H41,H42,z11,z21,z31,z41,T_r,...
                    sigma_or_L,sigma_or_R,sigma_cmc,s_delta_or);
                
%                 dt_gtsam=gtsam.time(jj)-gtsam.time(jj-1);
%             TimeGTSAM(jj)=dt_gtsam+TimeGTSAM(jj-1);
%             [insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P] = gtsamUpt(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,llhGTSAM(:,kk),Ro,ecc);
%             jj=jj+1;

                %% Slip Calculation
                slipCalculation
                %% GP process
                gpProcess
                %% Store detected slipped locations
                if abs(slipBL(1,odomUptCount))>0.2 || abs(slipBR(1,odomUptCount))>0.2 || abs(slipFL(1,odomUptCount))>0.2 || abs(slipFR(1,odomUptCount))>0.2
                    LLHcorrected1(:,cttr0)=insLLH(:,i);
                    cttr0=cttr0+1;
                end
            end % odom update
            %% Destroy H and z values for the next values
            H11=zeros(1,3);
            H12=zeros(1,3);
            H21=zeros(1,3);
            H31=zeros(1,3);
            H32=zeros(1,3);
            H24=zeros(1,3);
            H41=zeros(1,3);
            H42=zeros(1,3);
            z11=0;
            z21=0;
            z31=0;
            z41=0;
            counter=counter+1;
            kk=kk+1;
        end % odom update not available
 if gtsamUpdate()
        if TimeIMU(i) >= TimeGTSAM(jj)
            dt_gtsam=gtsam.time(jj)-gtsam.time(jj-1);
            TimeGTSAM(jj)=dt_gtsam+TimeGTSAM(jj-1);
            [insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P] = gtsamUpt(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,llhGTSAM(:,jj),Ro,ecc);
            jj=jj+1;
            
        end
 end
%  if gtsamUpdate()
%         if TimeIMU(i) >= TimeGPS(jj)
%             dt_gps=gpsECEF.time(jj)-gpsECEF.time(jj-1);
%             TimeGPS(jj)=dt_gps+TimeGPS(jj-1);
%             [insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P] = gtsamUpt(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,llhGPS(:,jj),Ro,ecc);
%             jj=jj+1;
%             
%         end
%  end
        %% Check if Zero Update is available with wheel velocity
        if abs(rearVel(kk))<0.005 && sign(frontRightVel(kk))*sign(frontLeftVel(kk))~=1% triggers zupt
            zeroUptCount=zeroUptCount+1;
            if zeroUpdate == true % && kk<50
              ZuptTime(:,zeroUptCount)=TimeIMU(i);

                [insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,postFitZero] = zeroUpd(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,omega_b_ib);
            end
            zCtr(i)=cttr3+offsetCtr;
            LLHcorrected(:,cttr3)=insLLH(:,i);
            cttr3=cttr3+1;
        else
            zCtr(i)=0;
            offsetCtr=offsetCtr+1;
        end % zero update not available
        %% Back Propagation
        if backProp()
            backPropagation
        else
            ba(1:3,i)=x_err(10:12);
            bg(1:3,i)=x_err(13:15);
        end
    end
    
    sig1(i)=3*sqrt(abs(P(1,1))); % 3 sigma values of att_x -roll
    sig2(i)=3*sqrt(abs(P(2,2))); % 3 sigma values of att_y -pitch
    sig3(i)=3*sqrt(abs(P(3,3))); % 3 sigma values of att_z -yaw
    
    sig4(i)=3*sqrt(abs(P(4,4))); % 3 sigma values of vel_x -forward
    sig5(i)=3*sqrt(abs(P(5,5))); % 3 sigma values of vel_y -left
    sig6(i)=3*sqrt(abs(P(6,6))); % 3 sigma values of vel_z -down
    
    sig7(i)=3*sqrt(abs(P(7,7))); % 3 sigma values of pos_x -latitude
    sig8(i)=3*sqrt(abs(P(8,8))); % 3 sigma values of pos_y -longitude
    sig9(i)=3*sqrt(abs(P(9,9))); % 3 sigma values of pos_z -height
    
    if gpsResults()
        gpsLonger(:,i)=[llhGPS(1,kk);llhGPS(2,kk);llhGPS(end,kk)];
    end
    x_State(:,i)=[insAtt(:,i);insVel(:,i);insLLH(:,i);ba(:,i);bg(:,i)];
    Cn2b_corr= eulr2dcm(insAtt(:,i));
    v_in(i)=[1,0,0]*Cn2b_corr*(insVel(:,i));
    insAttCorr(:,i)=dcm2eulr((eye(3)-skewsymm(x_err(1:3)))*Cn2b_corr');
    insVelCorr(:,i)=insVel(:,i)-x_err(4:6);
    insLLHCorr(:,i)=insLLH(:,i)-x_err(7:9);
%                    CompletePercentage=floor(i/length(Ax)*100);
%                if mod(CompletePercentage,10)==0
%                 CompletePercentage
%                end
end
if gpsResults()
gpsLonger(:,1)=gpsLonger(:,2);
end
figureGeneration
% figure;plot(slipBLTruth-slipBL)
% figure;plot(slipBLTruth);hold on;plot(slipBL)
% figure;plot(slipFLTruth-slipFL)
% figure;plot(tTodom-tTodom(1),rearVel)
% figure;plot(tTodom(1:end-1)-tTodom(1),v)
% figure;plot(tTimu-tTimu(1),v_in)
% figure;plot(tTodom(1:end-1)-tTodom(1),slipR)
% figure('Name','Analysis2','WindowStyle','docked');
