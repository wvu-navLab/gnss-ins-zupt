%% Differential Wheel Rover Low Level Navigation post processing script
% Authors: Cagri Kilic and Jason Gross
% Date: September 2018
%%  -----------------------------------------------------------------------
clc;
clear;
load pebblevsgrass\test20run.mat % LshapeData.mat %ForwardDrive.mat
%% odometry calculations
odom
%% initialize variables
MainRunInit2 % LshapeInit.mat %ForwardDriveInit.mat
slipCounter=0;
case1=false;%imu only
case2=true;%imu+predictionR
case3=false;%imu+predictionFixed
flag=true;
gpFlag=false;
stopFlag=false;
startRecording=0;
stopRecording=0;
utc=1;
%% Double Low Pass Filter
% dema
for i=2:L
    dtIMU=tTimu(i)-tTimu(i-1); % calculates imu delta time from rostomat output tTimu
    TimeIMU(i)=dtIMU+TimeIMU(i-1); % creates imu time from delta time starting from zero
    %%  -------------------------------------------------------------------
    %% IMU mounting orientation for BadgerRover
    omega_b_ib=[Gx(i),Gy(i),Gz(i)]-bg(:,i-1)'; %rad/s IMU gyro outputs minus estimated gyro bias
    f_ib_b = [Ax(i),Ay(i),Az(i)]-ba(:,i-1)'; %m/s^2 IMU acceleration minus estimated acce bias
    v_ib_b= f_ib_b* dtIMU; %m/s acceleration times delta IMU = velocity
    %% --------------------------------------------------------------------
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
    %% --------------------------------------------------------------------
    %% Error State Model--eq 14.63
    [STM] = insErrorStateModel_LNF(R_EPlus,R_N,insLLH(:,i),insVel(:,i),dtIMU,Cb2nPlus,omega_ie,omega_n_in,f_ib_b);
    %% Propagation of the Error
    x_err=STM*x_err;
    %% Q matrix --
    F21= -skewsymm(Cb2nPlus*(f_ib_b'));
    Q=getQins(F21,Cb2nPlus,insLLH(:,i),R_N,R_E,dtIMU);
    
    [~,q] = chol(Q);
    if q ~= 0
        %     disp('Q matrix is not positive definite')
        %     i
    end
    %% P matrix
    P = STM*P*STM' + Q; %call this step
    [~,p] = chol(P);
    if p ~= 0
        %     disp('P matrix is not positive definite')
        %     i
    end
    STM_fixed=STM;
    Q_fixed=Q;
    P_fixed=P;
    
    if odomUpdate()
        %% --------------------------------------------------------------------
        Cn2bPlus=Cb2nPlus';
        omega_b_ie=Cn2bPlus*omega_n_ie;
        omega_b_ei=-omega_b_ie;
        omega_b_eb=omega_b_ei+omega_b_ib';
        %% Measurement Matrix -- integration part for INS -- eq 16.48
        H11= H11+ [1,0,0]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H12= H12+ [1,0,0]*Cn2bPlus*dtIMU;
        H21= H21+ sin(insAtt(2,i))*[0,cos(insAtt(1,i)),sin(insAtt(1,i))]*Cn2bPlus*dtIMU;
        H31= H31+ [0,1,0]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H32= H32+ [0,1,0]*Cn2bPlus*dtIMU;
        H41= H41+ [0,0,1]*Cn2bPlus*skewsymm(insVel(:,i))*dtIMU;
        H42= H42+ [0,0,1]*Cn2bPlus*dtIMU;
        %% Measurement Innovation -- integration part for INS -- eq 16.42
        z11=z11+[1,0,0]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;
        z21=z21+cos(insAtt(2,i))*dtIMU;
        z31=z31+[0,1,0]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;
        z41=z41+[0,0,1]*(Cn2bPlus*insVel(:,i) + skewsymm(omega_b_eb)*[-A,0,0]')*dtIMU;
        % z and H values are integrated for IMU only above. When the Odometry
        % update is available (tTimu(i)>=tTodom(kk)) integrated values will be
        % updated by odometry measurements, used in the filter and destroyed.
    end
    if nonHolo()
        [insAtt(:,i),insVel(:,i),insLLH(:,i),x_err,P]= nonHolonomic(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,omega_n_ie,omega_b_ib,A);
    end
    %% --------------------------------------------------------------------
    xState{1,i}=x_err;
    PStore{1,i}=P;
    STMStore{1,i}=STM+eye(15).*x_err;
    if kk<min(min(length(heading),length(lin_x))) % Only valid for post-processing
        
        %% ----------------------------------------------------------------
        if tTimu(i)>=tTodom(kk) % Odometry update is available
            bb(counter)=i; % counts for the number of `i` when the loop goes into odometry updates
            dt_odom=tTodom(kk)-tTodom(kk-1);
            %% ------------------------------------------------------------
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
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                slipCalculation                         %%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                
                if slipFR(:,odomUptCount) ~=0.0 && slipFR(:,odomUptCount) ~=-1 && slipFR(:,odomUptCount) ~=1
                    if flag()
                        disp('Driving starts')
                        saveCountOdom=odomUptCount;
                        startRecording=saveCountOdom+30;
                        stopRecording=startRecording+200;
                        flag=false;
                    elseif gpFlag()
                        %                         disp('Re-Processing starts while driving')
                    end
                    
                    
                    if odomUptCount > startRecording && odomUptCount < stopRecording
                        slipInterval(odomUptCount-startRecording)=slipFR(odomUptCount);
                        slipTime(odomUptCount-startRecording)=tTodom(odomUptCount)-tTodom(1);
                        slipCounter=slipCounter+1;
                    end
                    
                    % Save slipInterval and slipTime and call them from a python script
                    
                    if odomUptCount==stopRecording
                        writematrix(vertcat(slipTime,slipInterval)', 'cagriGPcodes\slipForecastPython\slipVal.csv','Delimiter','comma')
                        H_fixed=postFitOdom.H;
                        P_pred=P_fixed;
                        timeCountSodom(1)=tTodom(odomUptCount);
                        timeCountSimu(1)=tTimu(i);
                        savePos=insLLH(:,i);
                        if gpFlag()
                            disp('New data available!')
                            disp('GP processing')
%                             pause()
                        else disp('GP processing')
                        end
                        % Process the slip values and slip time and give predicted sigma and mean values from GP
                        [predSlip_sigma,predSlip_mean]=GP(startRecording,stopRecording);
                        %                     figure;plot(predSlip_mean+3*predSlip_sigma);hold on; plot(predSlip_mean-3*predSlip_sigma);hold on;plot(slipInterval);hold on;plot(predSlip_mean);
                        
                        startRecording=stopRecording;
                        stopRecording=startRecording+200;
                        gpFlag=true;
                        % Map the slip values and get predicted velocity sigma, take average velocity
                        % mean([median(abs(velBackLeft)),median(abs(velBackRight)),median(abs(velFrontLeft)),median(abs(velFrontRight))])
                        
                        %                     if case1()
                        %                         for slip_i=1:25*length(predSlip_mean)
                        %                             P_pred = STM_fixed*P_pred*STM_fixed' + Q_fixed;
                        %                             sig7_pred(slip_i)=3*sqrt(abs(P_pred(7,7))); % 3 sigma values of pos_x -latitude
                        %                             sig8_pred(slip_i)=3*sqrt(abs(P_pred(8,8))); % 3 sigma values of pos_y -longitude
                        %                             sig9_pred(slip_i)=3*sqrt(abs(P_pred(9,9))); % 3 sigma values of pos_z -height
                        %                             P_pred_pos(:,slip_i)=[P_pred(7,7);P_pred(8,8);P_pred(9,9)];
                        %                             if slip_i<25*length(predSlip_mean)
                        %                                 timeCountSimu(slip_i+1)=timeCountSimu(slip_i)+0.004;
                        %                             end
                        %                         end
                        %                     end
                        
                        if case2()
                            for slip_i=1:25*length(predSlip_mean)
                                P_pred = STM_fixed*P_pred*STM_fixed' + Q_fixed;
                                if mod(slip_i,25)==0
                                    
                                    [chi_UT_est,chi_UT_est_cov] = UTfun(predSlip_mean(utc),predSlip_sigma(utc));
                                    rearVel3(utc)=chi_UT_est;
                                    rearVelCov(utc)=chi_UT_est_cov;
                                    countS(utc)=utc;
                                    if utc<length(predSlip_mean)
                                        timeCountSodom(utc+1)=timeCountSodom(utc)+0.1;
                                    end
                                    
                                    R_IP(:,utc)= diag([max(0.00045,rearVelCov(utc)^2),0,0,0;0,max(0.1152,rearVelCov(utc)^2),0,0;0,0,0.0025,0;0,0,0,0.0025]);%IMU+PredictionR
                                    R_IPA= diag([0.00045,0,0,0;0,0.1152,0,0;0,0,0.0025,0;0,0,0,0.0025]); %IMU+PredictedFixed
                                    [P_pred]=odomUptCov2(P_pred,H_fixed,R_IP(:,utc).*eye(4));
                                    utc=utc+1;
                                end
                                sig7_pred(slip_i)=3*sqrt(abs(P_pred(7,7))); % 3 sigma values of pos_x -latitude
                                sig8_pred(slip_i)=3*sqrt(abs(P_pred(8,8))); % 3 sigma values of pos_y -longitude
                                sig9_pred(slip_i)=3*sqrt(abs(P_pred(9,9))); % 3 sigma values of pos_z -height
                                P_pred_pos(:,slip_i)=[P_pred(7,7);P_pred(8,8);P_pred(9,9)];
                                if slip_i<25*length(predSlip_mean)
                                    timeCountSimu(slip_i+1)=timeCountSimu(slip_i)+0.004;
                                end
                                
                                xyzSlip=llh2xyz(savePos);
                                xyzSlip_3P=llh2xyz([savePos(1)-sig7_pred(slip_i);savePos(2)-sig8_pred(slip_i);savePos(3)-sig9_pred(slip_i)]);
                                xyzSlip3P=llh2xyz([savePos(1)+sig7_pred(slip_i);savePos(2)+sig8_pred(slip_i);savePos(3)+sig9_pred(slip_i)]);
                                ENUSlip=xyz2enu(xyzSlip,xyzInit);
                                ENU3PSlip=xyz2enu(xyzSlip3P,xyzInit);
                                ENU_3PSlip=xyz2enu(xyzSlip_3P,xyzInit);
                                xy_errSlip=ENU3PSlip(1:2,:)-ENUSlip(1:2,:);
                                dPxSlip=sqrt(xy_errSlip(1:2,:)'*xy_errSlip(1:2,:));
                                
                                if dPxSlip>3 && dPxSlip<3.001
                                    fprintf('command stop after %f (s) driving, horizontal err pred: %f (m) current driving time: %f (s)\n',timeCountSimu(slip_i+1)-tTimu(1),dPxSlip,tTimu(i)-tTimu(1))
                                    stopFlag=true;
                                    break;
                                    %                                 odomUptCount
                                end
                            end
                            utc=1;
                            
                        end
                        fprintf('slip prediction is %f (m) after 90(s) from current driving time %f (s)\n',dPxSlip,timeCountSimu(1)-tTimu(1));
                        if case3()
                            for slip_i=1:25*length(predSlip_mean)
                                P_pred = STM_fixed*P_pred*STM_fixed' + Q_fixed;
                                if mod(slip_i,25)==0
                                    [P_pred]=odomUptCov2(P_pred,H_fixed,R_IPA);
                                end
                                sig7_pred(slip_i)=3*sqrt(abs(P_pred(7,7))); % 3 sigma values of pos_x -latitude
                                sig8_pred(slip_i)=3*sqrt(abs(P_pred(8,8))); % 3 sigma values of pos_y -longitude
                                sig9_pred(slip_i)=3*sqrt(abs(P_pred(9,9))); % 3 sigma values of pos_z -height
                                P_pred_pos(:,slip_i)=[P_pred(7,7);P_pred(8,8);P_pred(9,9)];
                                if slip_i<25*length(predSlip_mean)
                                    timeCountSimu(slip_i+1)=timeCountSimu(slip_i)+0.004;
                                end
                            end
                        end
                        
                        
                        
                        
                        % %                     figure;
                        %                     fill( [countS(1:end) fliplr(countS(1:end))],  [rearVel3(1:end)-3*rearVelCov(1:end) fliplr(rearVel3(1:end)+3*rearVelCov(1:end))], 'r','DisplayName','3\sigma Covariance Hull' );
                        %                     hold on;
                        %                     plot(rearVel3,'b')
                        %                     hold on;
                        %                     plot(v(62:362))
                        %                     hold on;
                        %                     plot(rearVel(62:362))
                        
                    end
                    %                 elseif slipFR(:,odomUptCount) == 0.0
                    %                     disp('Rover wheels are not moving!')
                elseif slipFR(:,odomUptCount) ==-1
                    disp('Rover throttled up! 100% slippage')
                elseif slipFR(:,odomUptCount) ==1
                    disp('Rover throttled down! 100% slippage')
                end
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                if abs(slipBL(1,odomUptCount))>0.2 || abs(slipBR(1,odomUptCount))>0.2 || abs(slipFL(1,odomUptCount))>0.2 || abs(slipFR(1,odomUptCount))>0.2
                    LLHcorrected1(:,cttr0)=insLLH(:,i);
                    cttr0=cttr0+1;
                end
            end % odom update
            % % % ------------------------------------------------------------
            % destroy H and z values for the next values
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
        %% ----------------------------------------------------------------
        if abs(rearVel(kk))<0.005% triggers zupt
            zeroUptCount=zeroUptCount+1;
            if zeroUpdate()
                [insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,postFitZero] = zeroUpd(insVel(:,i),insAtt(:,i),insLLH(:,i),x_err,P,omega_b_ib);
            end
            zCtr(i)=cttr3+offsetCtr;
            LLHcorrected(:,cttr3)=insLLH(:,i);
            cttr3=cttr3+1;
        else
            zCtr(i)=0;
            offsetCtr=offsetCtr+1;
        end % zero update not available
        
        if backProp()
            if zCtr(i)-zCtr(i-1) < 0 % checks the zero update counter diff. If it is > 0 ZeroUpdate applied
                doBackProp=false;
                for j=i-1:-1:2
                    if zCtr(j)-zCtr(j-1)<0
                        doBackProp=true;
                        lastZindex=j;
                        break;
                    end
                end
                if doBackProp
                    x_err_s=xState{1,i};
                    P_s=PStore{1,i};
                    STM_s=STMStore{1,i};
                    for dd=i:-1:lastZindex
                        [P_s,x_err_s] = smoothback(PStore{dd-1},PStore{dd},STMStore{dd-1},xState{dd-1},xState{dd},x_err_s,P_s,STM_s);
                        insVel(:,dd)=insVel(:,dd)-x_err_s(4:6);
                        insLLH(:,dd)=insLLH(:,dd)-x_err_s(7:9);
                        Cn2b_propBack= eulr2dcm(insAtt(:,dd));
                        insAtt(:,dd)=dcm2eulr((eye(3)-skewsymm(x_err_s(1:3)))*Cn2b_propBack');
                        ba(1:3,dd)=x_err_s(10:12);
                        bg(1:3,dd)=x_err_s(10:12);
                    end
                else
                    ba(1:3,i)=x_err(10:12); % acce bias, this value will be removed from IMU acce output
                    bg(1:3,i)=x_err(13:15); % gyro bias, this value will be removed from IMU gyro output
                end % doBackProp
            end % if ZeroUpdate applied
        else
            ba(1:3,i)=x_err(10:12); % acce bias, this value will be removed from IMU acce output
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
end
figureGeneration

% figure;plot(tTodom-tTodom(1),rearVel)
% figure;plot(tTodom(1:end-1)-tTodom(1),v)
% figure;plot(tTimu-tTimu(1),v_in)
% figure;plot(tTodom(1:end-1)-tTodom(1),slipR)