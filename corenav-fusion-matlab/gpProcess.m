                if slipFR(:,odomUptCount) ~=0.0 && slipFR(:,odomUptCount) ~=-1 && slipFR(:,odomUptCount) ~=1
                    if flag()
                        disp('Driving starts')
                        saveCountOdom=odomUptCount;
%                         startRecording=saveCountOdom+10;
%                         stopRecording=startRecording+150;
                        startRecording=saveCountOdom+400;
                        stopRecording=startRecording+800;
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
%                         writematrix(vertcat(slipTime,slipInterval)', 'gaussianProcess\slipForecastPython\slipVal.csv','Delimiter','comma')
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
                         figure;plot(predSlip_mean+3*predSlip_sigma);hold on; plot(predSlip_mean-3*predSlip_sigma);hold on;plot(slipInterval);hold on;plot(predSlip_mean);
                        
%                         startRecording=stopRecording;
%                         stopRecording=startRecording+200;
                        gpFlag=true;

                            for slip_i=1:5*length(predSlip_mean)
                                P_pred = STM_fixed*P_pred*STM_fixed' + Q_fixed;
                                if mod(slip_i,5)==0
                                    
                                    [chi_UT_est,chi_UT_est_cov] = UTfun(predSlip_mean(utc),predSlip_sigma(utc));
                                    rearVel3(utc)=chi_UT_est;
                                    rearVelCov(utc)=chi_UT_est_cov;
                                    countS(utc)=utc;
                                    if utc<length(predSlip_mean)
                                        timeCountSodom(utc+1)=timeCountSodom(utc)+0.1;
                                    end
                                    R_IP_1=[0.5,0.5,0.0,0.0;
                                            1/0.685,-1/0.685,0.0,0.0;
                                            0.0,0.0,1.0,0.0;
                                            0.0,0.0,0.0,1.0];

                                    R_IP_2= [max(0.15^2,rearVelCov(utc)^2), 0.0,0.0,0.0;
                                    0, max(0.15^2,rearVelCov(utc)^2),0.0,0.0;
                                    0, 0, max(0.15^2,rearVelCov(utc)^2),0.0;
                                    0,0,0,0.15^2];
%                                     R_IP_2= [(0.15^2), 0.0,0.0,0.0;
%                                     0, 0.15^2,0.0,0.0;
%                                     0, 0, 0.15^2,0.0;
%                                     0,0,0,0.05^2];

                                    R_IP(:,utc)= diag(R_IP_1*R_IP_2*R_IP_1');%IMU+PredictionR
%                                     R_IP(:,utc)= diag([max(0.00045,rearVelCov(utc)^2),0,0,0;0,max(0.1152,rearVelCov(utc)^2),0,0;0,0,0.0025,0;0,0,0,0.0025]);%IMU+PredictionR

%                                     R_IPA= diag([0.00045,0,0,0;0,0.1152,0,0;0,0,0.0025,0;0,0,0,0.0025]); %IMU+PredictedFixed
                                    [P_pred]=odomUptCov2(P_pred,H_fixed,R_IP(:,utc).*eye(4));
                                    utc=utc+1;
                                end
                                sig7_pred(slip_i)=3*sqrt(abs(P_pred(7,7))); % 3 sigma values of pos_x -latitude
                                sig8_pred(slip_i)=3*sqrt(abs(P_pred(8,8))); % 3 sigma values of pos_y -longitude
                                sig9_pred(slip_i)=3*sqrt(abs(P_pred(9,9))); % 3 sigma values of pos_z -height
                                P_pred_pos(:,slip_i)=[P_pred(7,7);P_pred(8,8);P_pred(9,9)];
                                if slip_i<5*length(predSlip_mean)
                                    timeCountSimu(slip_i+1)=timeCountSimu(slip_i)+0.02;
                                end
                                
                                xyzSlip=llh2xyz(savePos);
                                xyzSlip_3P=llh2xyz([savePos(1)-sig7_pred(slip_i);savePos(2)-sig8_pred(slip_i);savePos(3)-sig9_pred(slip_i)]);
                                xyzSlip3P=llh2xyz([savePos(1)+sig7_pred(slip_i);savePos(2)+sig8_pred(slip_i);savePos(3)+sig9_pred(slip_i)]);
                                ENUSlip=xyz2enu(xyzSlip,xyzInit);
                                ENU3PSlip=xyz2enu(xyzSlip3P,xyzInit);
                                ENU_3PSlip=xyz2enu(xyzSlip_3P,xyzInit);
                                xy_errSlip=ENU3PSlip(1:2,:)-ENUSlip(1:2,:);
                                dPxSlip=sqrt(xy_errSlip(1:2,:)'*xy_errSlip(1:2,:));
                                
                                if dPxSlip>3.0 && dPxSlip<3.001
                                    fprintf('command stop after %f (s) driving, horizontal err pred: %f (m) current driving time: %f (s)\n',timeCountSimu(slip_i+1)-tTimu(1),dPxSlip,tTimu(i)-tTimu(1))
                                    stopFlag=true;
%                                     break;
                                    %                                 odomUptCount
                                end
                            end
                            utc=1;
                            
%                         end
                        fprintf('slip prediction is %f (m) after 60(s) from current driving time %f (s)\n',dPxSlip,timeCountSimu(1)-tTimu(1));
                        
                    end
                    %                 elseif slipFR(:,odomUptCount) == 0.0
                    %                     disp('Rover wheels are not moving!')
                elseif slipFR(:,odomUptCount) ==-1
%                     disp('Rover throttled up! 100% slippage')
                elseif slipFR(:,odomUptCount) ==1
%                     disp('Rover throttled down! 100% slippage')
                end

               