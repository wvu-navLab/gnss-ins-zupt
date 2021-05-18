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