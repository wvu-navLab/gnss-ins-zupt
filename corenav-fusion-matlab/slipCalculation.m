                Cn2bPlus=Cb2nPlus';
                v(odomUptCount)=[1,0,0]*Cn2bPlus*(insVel(:,i));
                
                vX(odomUptCount)=[1,0,0]*Cn2bPlus*(insVel(:,i));
                vY(odomUptCount)=[0,1,0]*Cn2bPlus*(insVel(:,i));
%                 vGPSX(odomUptCount)=[1,0,0]*Cn2bPlus*(gpsVely1Shift(:,odomUptCount));
%                 vGPSX(odomUptCount)=[1,0,0]*Cn2bPlus*(y2sort1(:,odomUptCount));
%                 vGPSX(odomUptCount)=[0,0,0]*Cn2bPlus*(gpsVely1Orig(:,odomUptCount));
                rollIMU(odomUptCount)=insAtt(1,i);
                pitchIMU(odomUptCount)=insAtt(2,i);
                yawIMU(odomUptCount)=insAtt(3,i);
                sideslip(odomUptCount)=atan(vY(odomUptCount)/vX(odomUptCount));
                vGPSY=atan(gpsVely1Shift(2,1:end-1)./gpsVely1Shift(1,1:end-1));
                %sideslip = atan(lateral_Vel/tangential_Vel)
                if abs(vX(odomUptCount))<0.01 || abs(vY(odomUptCount))<0.01
                    sideslip(odomUptCount)=0.0;
                end

                slipBL(:,odomUptCount)=((velBackLeft(odomUptCount))-(v(odomUptCount)))/(velBackLeft(odomUptCount));
                slipBLTruth(:,odomUptCount)=((velBackLeft(odomUptCount))-(vGPSX(odomUptCount)))/(velBackLeft(odomUptCount));

                if velBackLeft(odomUptCount)==0
                    slipBL(:,odomUptCount) = 0;
                    slipBLTruth(:,odomUptCount) = 0;
                end
                if slipBL(:,odomUptCount) < -1
                    slipBL (:,odomUptCount) = -1;
                end
                if slipBLTruth(:,odomUptCount) < -1
                    slipBLTruth (:,odomUptCount) = -1;
                end
                if slipBL(:,odomUptCount) > 1
                    slipBL (:,odomUptCount) = 1;
                end
                if slipBLTruth(:,odomUptCount) > 1
                    slipBLTruth(:,odomUptCount) = 1;
                end                
                
                slipBR(:,odomUptCount)=((velBackRight(odomUptCount))-(v(odomUptCount)))/(velBackRight(odomUptCount));
                slipBRTruth(:,odomUptCount)=((velBackRight(odomUptCount))-(vGPSX(odomUptCount)))/(velBackRight(odomUptCount));
                if velBackRight(odomUptCount)==0
                    slipBR(:,odomUptCount) = 0;
                    slipBRTruth(:,odomUptCount) = 0;
                end
                if slipBR(:,odomUptCount) < -1
                    slipBR (:,odomUptCount) = -1;
                end
                if slipBR(:,odomUptCount) > 1
                    slipBR (:,odomUptCount) = 1;
                end
                 if slipBRTruth(:,odomUptCount) < -1
                    slipBRTruth(:,odomUptCount) = -1;
                end
                if slipBRTruth(:,odomUptCount) > 1
                    slipBRTruth(:,odomUptCount) = 1;
                end               
                
                slipFL(:,odomUptCount)=((velFrontLeft(odomUptCount))-(v(odomUptCount)))/(velFrontLeft(odomUptCount));
                slipFLTruth(:,odomUptCount)=((velFrontLeft(odomUptCount))-(vGPSX(odomUptCount)))/(velFrontLeft(odomUptCount));
                
                if velFrontLeft(odomUptCount)==0
                    slipFL(:,odomUptCount) = 0;
                    slipFLTruth(:,odomUptCount) = 0;
                end
                
                if slipFL(:,odomUptCount) < -1
                    slipFL (:,odomUptCount) = -1;
                end
                if slipFL(:,odomUptCount) > 1
                    slipFL (:,odomUptCount) = 1;
                end
                if slipFLTruth(:,odomUptCount) < -1
                    slipFLTruth (:,odomUptCount) = -1;
                end
                if slipFLTruth(:,odomUptCount) > 1
                    slipFLTruth(:,odomUptCount) = 1;
                end
                
                slipFR(:,odomUptCount)=((velFrontRight(odomUptCount))-(v(odomUptCount)))/(velFrontRight(odomUptCount));
                slipFRTruth(:,odomUptCount)=((velFrontRight(odomUptCount))-(vGPSX(odomUptCount)))/(velFrontRight(odomUptCount));
                if velFrontRight(odomUptCount)==0
                    slipFR(:,odomUptCount) = 0;
                    slipFRTruth(:,odomUptCount) = 0;
                end
                
                if slipFR(:,odomUptCount) < -1
                    slipFR (:,odomUptCount) = -1;
                end
                if slipFR(:,odomUptCount) > 1
                    slipFR (:,odomUptCount) = 1;
                end
                if slipFRTruth(:,odomUptCount) < -1
                    slipFRTruth(:,odomUptCount) = -1;
                end
                if slipFRTruth(:,odomUptCount) > 1
                    slipFRTruth(:,odomUptCount) = 1;
                end
%                 slip(:,odomUptCount)=max(max(slipFR(:,odomUptCount),slipFL(:,odomUptCount)),max(slipBL(:,odomUptCount),slipBR(:,odomUptCount)));